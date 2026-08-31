# REST API for remote car control and testing
# Runs in separate thread, shares state with game loop

from flask import Flask, jsonify, request, send_file
from threading import Thread, Lock
import io
import time
from typing import Dict, Any

from .obstacles import PlacementError


class GameAPI:
    """REST API for controlling the game and reading state."""
    
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config['JSON_SORT_KEYS'] = False
        
        # Shared state (thread-safe with lock)
        self.lock = Lock()
        self.game_state: Dict[str, Any] = {}
        self.control_input: Dict[str, bool] = {
            'accelerate': False,
            'brake': False,
            'steer_left': False,
            'steer_right': False,
            'blinker_left': False,
            'blinker_right': False,
            'uturn': False,
        }
        self.commands: Dict[str, Any] = {}  # For one-shot commands (teleport, etc.)
        self.screenshot_buffer: bytes = None
        
        # Obstacle system (docs/OBSTACLES.md): wired in by the game via
        # set_obstacles(). Placement/removal goes through the SAME logic as
        # the palette UI.
        self.obstacle_manager = None
        self.obstacle_network = None
        
        self._setup_routes()
    
    def _setup_routes(self):
        """Setup all API endpoints."""
        
        @self.app.route('/health', methods=['GET'])
        def health():
            """Health check."""
            return jsonify({'status': 'ok', 'timestamp': time.time()})
        
        @self.app.route('/state', methods=['GET'])
        def get_state():
            """Get current game state."""
            with self.lock:
                return jsonify(self.game_state)

        @self.app.route('/segment/<int:idx>', methods=['GET'])
        def segment_geometry(idx):
            """Geometry of one road segment (pixels, 0-based index).

            Used by the e2e suite to place the red end flag on the NODE
            SIDE of the exit arm - where the car actually enters it.
            """
            net = self.obstacle_network
            if net is None or not (0 <= idx < len(net.segments)):
                return jsonify({'error': 'unknown segment'}), 404
            s = net.segments[idx]
            return jsonify({
                'idx': idx,
                'x1': s.x1, 'y1': s.y1, 'x2': s.x2, 'y2': s.y2,
                'length': s.length, 'width': s.width, 'oneway': s.oneway,
            })
        
        @self.app.route('/control', methods=['POST'])
        def control():
            """Send control inputs to car.
            
            Body: {
                "accelerate": bool,
                "brake": bool,
                "steer_left": bool,
                "steer_right": bool,
                "blinker_left": bool,
                "blinker_right": bool,
                "uturn": bool   (one-shot: request an on-site U-turn / Wenden),
                "hazard": bool  (one-shot: true = hazard lights ON, false = OFF;
                                 they stay on for at least 5 s once triggered)
            }
            """
            data = request.get_json()
            with self.lock:
                for key in self.control_input.keys():
                    if key in data:
                        self.control_input[key] = bool(data[key])
                # Hazard is a one-shot command (both on AND off are explicit),
                # so it goes through the commands channel, not control_input.
                if 'hazard' in data:
                    self.commands['hazard'] = bool(data['hazard'])
            return jsonify({'ok': True, 'control': self.control_input})
        
        @self.app.route('/teleport', methods=['POST'])
        def teleport():
            """Teleport car to location.
            
            Body: {"random": true}
                  or {"start_point": "corner_right_entry"} (synthetic test maps only,
                      see GET /start_points for available names)
            """
            data = request.get_json()
            with self.lock:
                self.commands['teleport'] = data
            return jsonify({'ok': True, 'command': 'teleport', 'params': data})
        
        @self.app.route('/flags', methods=['POST'])
        def flags():
            """Set/clear the test start/end flags (visual confirmation
            markers drawn on the map, to the RIGHT of the road).

            Body: {"green": [x, y, heading_deg] | null,
                   "red": [segment_idx, progress] | null}
            Green is a world position (usually the car's); red is the
            expected END as segment index + progress along it - the game
            loop resolves it to a map position once the route covers that
            segment, so the end flag is visible from the start of the
            test. The renderer offsets both past the right kerb itself.
            """
            data = request.get_json(silent=True) or {}
            with self.lock:
                self.commands['flags'] = data
            return jsonify({'ok': True, 'command': 'flags', 'params': data})

        @self.app.route('/label', methods=['POST'])
        def label():
            """Set (or clear) a short text label shown in the HUD - handy for
            test scripts to show which test/map-tile is currently running.
            
            Body: {"text": "2/3"} or {"text": null} / {} to clear.
            """
            data = request.get_json(silent=True) or {}
            with self.lock:
                self.commands['label'] = data.get('text')
            return jsonify({'ok': True, 'command': 'label', 'params': data})
        
        @self.app.route('/start_points', methods=['GET'])
        def start_points():
            """List named deterministic start points (synthetic test maps only).
            Empty if the currently loaded map defines none (e.g. real OSM data).
            """
            with self.lock:
                return jsonify(self.game_state.get('start_points', {}))
        
        @self.app.route('/toggle', methods=['POST'])
        def toggle():
            """Toggle features.
            
            Body: {
                "breadcrumbs": bool,
                "validator": bool,
                "mode": "bicycle" | "free"
            }
            """
            data = request.get_json()
            with self.lock:
                self.commands['toggle'] = data
            return jsonify({'ok': True, 'command': 'toggle', 'params': data})
        
        @self.app.route('/screenshot', methods=['GET'])
        def screenshot():
            """Get current frame as PNG."""
            with self.lock:
                if self.screenshot_buffer is None:
                    return jsonify({'error': 'No screenshot available'}), 404
                
                return send_file(
                    io.BytesIO(self.screenshot_buffer),
                    mimetype='image/png',
                    as_attachment=False,
                    download_name='frame.png'
                )
        
        @self.app.route('/wait', methods=['POST'])
        def wait():
            """Wait for condition or timeout.
            
            Body: {
                "condition": "segment_changed" | "speed_reached" | "position_reached",
                "value": Any,
                "timeout": float (seconds)
            }
            
            Returns when condition met or timeout.
            """
            data = request.get_json()
            start_time = time.time()
            timeout = data.get('timeout', 5.0)
            condition = data.get('condition')
            value = data.get('value')
            
            while time.time() - start_time < timeout:
                with self.lock:
                    state = self.game_state.copy()
                
                # Check condition
                if condition == 'segment_changed' and state.get('segment') != value:
                    return jsonify({'ok': True, 'condition_met': True, 'state': state})
                elif condition == 'speed_reached' and state.get('speed', 0) >= value:
                    return jsonify({'ok': True, 'condition_met': True, 'state': state})
                elif condition == 'position_reached':
                    x, y = value['x'], value['y']
                    dist = ((state.get('x', 0) - x) ** 2 + (state.get('y', 0) - y) ** 2) ** 0.5
                    if dist < value.get('threshold', 10):
                        return jsonify({'ok': True, 'condition_met': True, 'state': state})
                
                time.sleep(0.05)
            
            return jsonify({'ok': False, 'condition_met': False, 'timeout': True})
        
        @self.app.route('/reset', methods=['POST'])
        def reset():
            """Reset control inputs to default (all false)."""
            with self.lock:
                for key in self.control_input.keys():
                    self.control_input[key] = False
            return jsonify({'ok': True, 'control': self.control_input})
        
        @self.app.route('/obstacles', methods=['GET'])
        def obstacles_list():
            """List placed obstacles.
            
            Returns: [{id, type, color, x, y, heading}, ...]
            (x/y in world coordinates, same as in saved layouts; heading
            is the auto-aligned lane direction, never a client input.)
            """
            if self.obstacle_manager is None:
                return jsonify({'error': 'obstacle system not available'}), 404
            return jsonify(self.obstacle_manager.snapshot_dicts())
        
        @self.app.route('/obstacles', methods=['POST'])
        def obstacles_place():
            """Place an obstacle (same placement logic as the palette UI).
            
            Body: {"type": "car", "color": "blue|yellow|white",
                   "x": <world px>, "y": <world px>}
            Returns 201 + the created obstacle (id + computed heading);
            4xx if the point is off-road or the request is invalid.
            """
            if self.obstacle_manager is None:
                return jsonify({'error': 'obstacle system not available'}), 404
            data = request.get_json(silent=True) or {}
            try:
                ob = self.obstacle_manager.place(
                    self.obstacle_network,
                    str(data.get("type", "car")),
                    str(data.get("color", "")),
                    float(data["x"]), float(data["y"]))
            except (KeyError, TypeError, ValueError) as e:
                return jsonify({'error': f'invalid request: {e}'}), 400
            except PlacementError as e:
                return jsonify({'error': str(e)}), 400
            return jsonify(ob.to_dict()), 201
        
        @self.app.route('/obstacles/<int:ob_id>', methods=['DELETE'])
        def obstacles_delete(ob_id: int):
            """Remove an obstacle; 404 if unknown id."""
            if self.obstacle_manager is None:
                return jsonify({'error': 'obstacle system not available'}), 404
            if not self.obstacle_manager.remove(ob_id):
                return jsonify({'error': f'no obstacle with id {ob_id}'}), 404
            return jsonify({'ok': True, 'id': ob_id})
    
    def set_obstacles(self, manager, network):
        """Wire the obstacle system (docs/OBSTACLES.md) into the API.
        Called by the game once the ObstacleManager exists."""
        self.obstacle_manager = manager
        self.obstacle_network = network
    
    def update_state(self, state: Dict[str, Any]):
        """Update game state (called from game loop)."""
        with self.lock:
            self.game_state.update(state)
    
    def get_control(self) -> Dict[str, bool]:
        """Get current control inputs (called from game loop)."""
        with self.lock:
            return self.control_input.copy()
    
    def clear_control(self, key: str):
        """Clear a single control flag (one-shot semantics, e.g. a blinker
        that was applied and should not re-trigger on the next frame)."""
        with self.lock:
            if key in self.control_input:
                self.control_input[key] = False
    
    def get_commands(self) -> Dict[str, Any]:
        """Get and clear pending commands (called from game loop)."""
        with self.lock:
            commands = self.commands.copy()
            self.commands.clear()
            return commands
    
    def update_screenshot(self, png_bytes: bytes):
        """Update screenshot buffer (called from game loop)."""
        with self.lock:
            self.screenshot_buffer = png_bytes
    
    def start(self, port: int = 5000, host: str = '127.0.0.1'):
        """Start API server in background thread."""
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)  # Suppress Flask startup messages
        
        def run():
            self.app.run(host=host, port=port, debug=False, use_reloader=False)
        
        thread = Thread(target=run, daemon=True, name='GameAPI')
        thread.start()
        print(f"🌐 REST API started on http://{host}:{port}")
        print(f"   Health check: http://{host}:{port}/health")
        print(f"   Game state:   http://{host}:{port}/state")
        print(f"   Control car:  POST http://{host}:{port}/control")
