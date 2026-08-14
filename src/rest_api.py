# REST API for remote car control and testing
# Runs in separate thread, shares state with game loop

from flask import Flask, jsonify, request, send_file
from threading import Thread, Lock
import io
import time
from typing import Dict, Any


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
        }
        self.commands: Dict[str, Any] = {}  # For one-shot commands (teleport, etc.)
        self.screenshot_buffer: bytes = None
        
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
        
        @self.app.route('/control', methods=['POST'])
        def control():
            """Send control inputs to car.
            
            Body: {
                "accelerate": bool,
                "brake": bool,
                "steer_left": bool,
                "steer_right": bool,
                "blinker_left": bool,
                "blinker_right": bool
            }
            """
            data = request.get_json()
            with self.lock:
                for key in self.control_input.keys():
                    if key in data:
                        self.control_input[key] = bool(data[key])
            return jsonify({'ok': True, 'control': self.control_input})
        
        @self.app.route('/teleport', methods=['POST'])
        def teleport():
            """Teleport car to location.
            
            Body: {"random": true} or {"segment": int, "progress": float}
            """
            data = request.get_json()
            with self.lock:
                self.commands['teleport'] = data
            return jsonify({'ok': True, 'command': 'teleport', 'params': data})
        
        @self.app.route('/toggle', methods=['POST'])
        def toggle():
            """Toggle features.
            
            Body: {
                "breadcrumbs": bool,
                "validator": bool,
                "mode": "rails" | "free"
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
    
    def update_state(self, state: Dict[str, Any]):
        """Update game state (called from game loop)."""
        with self.lock:
            self.game_state.update(state)
    
    def get_control(self) -> Dict[str, bool]:
        """Get current control inputs (called from game loop)."""
        with self.lock:
            return self.control_input.copy()
    
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
        def run():
            self.app.run(host=host, port=port, debug=False, use_reloader=False)
        
        thread = Thread(target=run, daemon=True, name='GameAPI')
        thread.start()
        print(f"🌐 REST API started on http://{host}:{port}")
        print(f"   Health check: http://{host}:{port}/health")
        print(f"   Game state:   http://{host}:{port}/state")
        print(f"   Control car:  POST http://{host}:{port}/control")
