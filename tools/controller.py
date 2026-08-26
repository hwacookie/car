#!/usr/bin/env python3
"""Car Controller — the driver's cockpit window.

A standalone process (its own pygame window) that talks to the running game
over the REST API, exactly like the test runner does. The game window stays
a pure world view; everything a DRIVER looks at lives here:

  * the dashboard - speed, mode, segment, accel/brake lamps, blinkers,
    hazard, OFF-ROAD / WRONG-SIDE warnings (fed by GET /state at 10 Hz), and
  * one of two control panels:

    --mode test   (default): runs the deterministic e2e suite and shows a
                             row of numbered scenario buttons (1..N).
                             Clicking a button - or typing its number and
                             pressing Enter - ABORTS the current test and
                             starts that one, then continues with the rest.

    --mode drive  : manual driving console for FREE mode: hold-buttons for
                    steering / gas / brake plus one-shot blinker stalks,
                    hazard and U-turn (mouse or keyboard), so a human can
                    drive the car while watching the world window. Press F
                    to switch the game between AI (BICYCLE) and manual
                    (FREE) driving.

Both panels feed the same control channel (POST /control) that the test
runner uses - the cockpit is just another client of the driving API.

Usage:
  python tools/controller.py                      # run the suite, with cockpit
  python tools/controller.py --tests 1-3          # only those scenarios
  python tools/controller.py --drive              # manual driving console
  python tools/controller.py --selftest           # scripted smoke test of the
                                                  # click/jump path (test mode)
                                                  # or gas/brake path (drive mode)
"""

import argparse
import os
import queue
import signal
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import pygame
import requests

from tests.test_turning import (API_URL, DETERMINISTIC_TESTS, TurnTester,
                                load_results, select_tests)

# --- Window / layout constants ---
W, H = 840, 540
BG = (24, 26, 30)
PANEL = (38, 41, 48)
PANEL_BORDER = (70, 74, 82)
TEXT = (230, 230, 235)
DIM = (150, 154, 162)
GREEN = (90, 200, 120)
RED = (230, 80, 80)
YELLOW = (240, 200, 70)
ORANGE = (240, 150, 60)
BLUE = (110, 170, 240)
BTN_IDLE_FILL = (52, 56, 64)
BTN_IDLE_BORDER = (90, 95, 105)

DRIVE_HINT = ("Hold: arrows/WASD drive · Q/E blinkers · H hazard · U uturn · F AI/manual — "
              "or click the pedals")


# --- Text rendering (PIL, same approach as src/renderer.py - pygame.font is
# unreliable on some platforms) ---

_FONT_CACHE = {}
_TEXT_CACHE = {}


def _font(size: int):
    from PIL import ImageFont
    f = _FONT_CACHE.get(size)
    if f is None:
        try:
            f = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", size)
        except Exception:
            f = ImageFont.load_default()
        _FONT_CACHE[size] = f
    return f


def make_text(s: str, size: int, color: tuple) -> pygame.Surface:
    key = (s, size, color)
    surf = _TEXT_CACHE.get(key)
    if surf is None:
        from PIL import Image, ImageDraw
        font = _font(size)
        tmp = Image.new("RGBA", (1, 1), (0, 0, 0, 0))
        bbox = ImageDraw.Draw(tmp).textbbox((0, 0), s, font=font)
        w = max(1, bbox[2] - bbox[0] + 2)
        h = max(1, bbox[3] - bbox[1] + 2)
        img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        ImageDraw.Draw(img).text((-bbox[0], -bbox[1]), s, fill=(*color, 255), font=font)
        surf = pygame.image.fromstring(img.tobytes(), img.size, img.mode)
        if len(_TEXT_CACHE) > 2048:
            _TEXT_CACHE.clear()
        _TEXT_CACHE[key] = surf
    return surf


def blit_centered(screen, s: str, size: int, color: tuple, cx: int, cy: int):
    surf = make_text(s, size, color)
    screen.blit(surf, (cx - surf.get_width() // 2, cy - surf.get_height() // 2))


class Controller:
    """The cockpit: dashboard + test buttons (test mode) or driving console
    (drive mode). Talks to the game over REST only."""

    def __init__(self, args):
        self.url = args.url
        self.mode = args.mode
        self.spec = args.tests
        self.selftest = args.selftest
        self.connected = False
        self.state: dict = {}
        self._exit_evt = threading.Event()
        self.events: "queue.Queue" = queue.Queue()
        self.frame = 0

        # --- Test-mode state (scenario buttons + suite worker) ---
        self.tester = TurnTester()
        if self.mode == 'test':
            # select_tests exits(2) on a bad spec - do it BEFORE opening the
            # window so a typo doesn't flash an empty cockpit.
            self.selected = select_tests(DETERMINISTIC_TESTS, self.spec)
        else:
            self.selected = []
        self.numbers = [i for i, _ in self.selected]
        self.btn_state = {n: 'idle' for n in self.numbers}  # idle|running|passed|failed
        self.tally = {'passed': 0, 'failed': 0, 'aborted': 0}
        self.status_line1 = "Waiting for the game API…"
        self.status_line2 = ""
        self._jump_lock = threading.Lock()
        self._jump_target: int | None = None
        self.suite_done = False
        self.exit_code = 0

        # --- Drive-mode state (manual console) ---
        self.held = {'accelerate': False, 'brake': False,
                     'steer_left': False, 'steer_right': False}
        self.hazard_on = False
        self.free_mode = False   # local mirror of the game's driver mode

        # --- Selftest bookkeeping ---
        self._st_phase = 0
        self._st_fired = False

        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Car Controller — cockpit")
        self.clock = pygame.time.Clock()
        # macOS: bring the window to the foreground like the game does.
        if sys.platform == "darwin":
            import subprocess
            subprocess.run([
                "osascript", "-e",
                f'tell application "System Events" to set frontmost of '
                f'(first process whose unix id is {os.getpid()}) to true',
            ], capture_output=True)
        self._layout()

    # --- Layout: button rects (rebuilt once; the window is fixed-size) ---

    def _layout(self):
        if self.mode == 'test':
            cols = 8
            bw, bh, gap = 88, 60, 10
            rows = (len(self.numbers) + cols - 1) // cols
            grid_w = cols * bw + (cols - 1) * gap
            x0 = (W - grid_w) // 2
            y0 = H - 84 - rows * (bh + gap)
            self.scenario_rects: dict[int, pygame.Rect] = {}
            for idx, n in enumerate(self.numbers):
                row, col = divmod(idx, cols)
                self.scenario_rects[n] = pygame.Rect(
                    x0 + col * (bw + gap), y0 + row * (bh + gap), bw, bh)
        else:
            y1, y2 = H - 268, H - 148
            self.drive_rects = {
                'steer_left': pygame.Rect(56, y1, 160, 96),
                'accelerate': pygame.Rect(236, y1, 170, 96),
                'brake': pygame.Rect(426, y1, 170, 96),
                'steer_right': pygame.Rect(616, y1, 168, 96),
            }
            self.one_shot_rects = {
                'blinker_left': pygame.Rect(56, y2, 176, 74),
                'blinker_right': pygame.Rect(252, y2, 176, 74),
                'hazard': pygame.Rect(448, y2, 156, 74),
                'uturn': pygame.Rect(624, y2, 160, 74),
            }

    # --- REST helpers (the cockpit is just another client of the API) ---

    def _post(self, path: str, payload: dict | None = None) -> bool:
        try:
            requests.post(f"{self.url}{path}", json=payload or {}, timeout=2)
            return True
        except requests.exceptions.RequestException as e:
            print(f"⚠️  POST {path} failed: {e}")
            return False

    def _set_held(self, key: str, val: bool):
        if self.held[key] == val:
            return
        self.held[key] = val
        self._post('/control', {key: val})

    def _toggle_hazard(self):
        self.hazard_on = not self.hazard_on
        self._post('/control', {'hazard': self.hazard_on})

    def _toggle_driver_mode(self):
        # FREE (human) <-> BICYCLE (AI), like the TAB key in the game window.
        self.free_mode = not self.free_mode
        mode = 'free' if self.free_mode else 'bicycle'
        if self._post('/toggle', {'mode': mode}):
            print(f"🔁 Driver mode → {mode.upper()}")

    # --- Test jump (button click / typed number) ---

    def request_jump(self, num: int):
        """Abort the current test and start scenario `num` (1-based)."""
        if num not in self.numbers:
            print(f"⚠️  No test {num} in the selected set "
                  f"({self.numbers[0]}..{self.numbers[-1]})")
            return
        with self._jump_lock:
            self._jump_target = num
        print(f"\n⏭️  JUMP requested → test {num}")

    def _peek_jump(self) -> bool:
        with self._jump_lock:
            return self._jump_target is not None

    def _take_jump(self) -> int | None:
        with self._jump_lock:
            t = self._jump_target
            self._jump_target = None
            return t

    # --- Background threads ---

    def _poll_loop(self):
        """Dashboard data: GET /state at 10 Hz."""
        while not self._exit_evt.is_set():
            try:
                r = requests.get(f"{self.url}/state", timeout=1)
                self.state = r.json()
                self.connected = True
            except requests.exceptions.RequestException:
                self.connected = False
            self._exit_evt.wait(0.1)

    def _suite_worker(self, results: dict):
        """Run the deterministic suite in a worker thread; button clicks are
        picked up via abort_check/take_jump (same process - no IPC needed)."""
        while not self.connected and not self._exit_evt.is_set():
            time.sleep(0.5)
        if self._exit_evt.is_set():
            return
        try:
            self.tester.run_deterministic_test(
                results=results, spec=self.spec,
                abort_check=self._peek_jump, take_jump=self._take_jump,
                on_event=lambda ev: self.events.put(ev))
            runnable = [r for r in self.tester.test_results if not r.get('aborted')]
            self.events.put({
                'type': 'suite_done',
                'all_passed': all(r['passed'] for r in runnable),
                'passed': sum(1 for r in runnable if r['passed']),
                'failed': sum(1 for r in runnable if not r['passed']),
                'aborted': len(self.tester.test_results) - len(runnable),
            })
        except BaseException as e:
            # SystemExit (e.g. map without start points) or a crash must not
            # kill the cockpit silently.
            self.events.put({'type': 'suite_done', 'error': f"{type(e).__name__}: {e}"})

    # --- Worker → UI event queue ---

    def _process_events(self):
        while True:
            try:
                ev = self.events.get_nowait()
            except queue.Empty:
                return
            t = ev['type']
            if t == 'start':
                self.btn_state[ev['index']] = 'running'
                self.status_line1 = (f"TEST {ev['index']}/{ev['total']} — "
                                     f"'{ev['start_point']}' → {ev['direction'].upper()} "
                                     f"@ {ev['speed_kmh']} km/h")
                self.status_line2 = ev.get('description') or ""
            elif t == 'done':
                if ev.get('aborted'):
                    self.btn_state[ev['index']] = 'idle'
                    self.tally['aborted'] += 1
                else:
                    self.btn_state[ev['index']] = 'passed' if ev['passed'] else 'failed'
                    self.tally['passed' if ev['passed'] else 'failed'] += 1
            elif t == 'jump':
                self.status_line2 = f"⏭ jumped from test {ev['from']} to test {ev['to']}"
            elif t == 'suite_done':
                self.suite_done = True
                if ev.get('error'):
                    print(f"\n❌ Suite error: {ev['error']}")
                    self.status_line1 = f"SUITE ERROR: {ev['error']}"
                    self.exit_code = 1
                else:
                    print(f"\n🏁 Suite finished: {ev['passed']} passed, "
                          f"{ev['failed']} failed, {ev['aborted']} aborted")
                    self.status_line1 = (f"SUITE DONE — {ev['passed']} passed, "
                                         f"{ev['failed']} failed, {ev['aborted']} aborted")
                    self.exit_code = 0 if ev['all_passed'] else 1
                # Let the final button colors be seen for a moment.
                self._close_at = time.time() + 2.5

    # --- Selftest (scripted smoke test of the cockpit's own paths) ---

    def _selftest(self, now: float):
        if self.selftest is None:
            return
        t = now - self.t0
        if self.mode == 'test':
            if not self._st_fired and t >= 6.0:
                self._st_fired = True
                print(f"\n🧪 SELFTEST: simulating a click on test button "
                      f"{self.selftest} at t={t:.1f}s (same code path as the mouse)")
                self.request_jump(self.selftest)
        else:
            # Drive mode: teleport a car, switch to FREE, press gas for 3 s,
            # then brake and verify the car actually stops.
            if self._st_phase == 0 and t >= 2.0:
                print("\n🧪 SELFTEST(drive): teleporting a car to 'corner_right_entry' "
                      "and switching to FREE mode...")
                self._post('/teleport', {'start_point': 'corner_right_entry',
                                         'progress': 0.5})
                self.free_mode = True
                self._post('/toggle', {'mode': 'free'})
                self._st_phase = 1
            elif self._st_phase == 1 and t >= 4.5:
                if not self.state.get('has_car'):
                    if t >= 9.0:
                        print("🧪 SELFTEST(drive): FAIL - no car after teleport")
                        self.exit_code = 1
                        self._close_at = time.time() + 1.0
                        self._st_phase = 9
                    return
                print("🧪 SELFTEST(drive): pressing GAS for 3 s...")
                self._set_held('accelerate', True)
                self._st_phase = 2
            elif self._st_phase == 2 and t >= 7.5:
                spd = self.state.get('speed_kmh', 0.0)
                ok = spd > 5.0
                print(f"🧪 SELFTEST(drive): speed after gas = {spd:.1f} km/h "
                      f"→ {'PASS' if ok else 'FAIL'}")
                self._set_held('accelerate', False)
                self._set_held('brake', True)
                self._st_phase = 3
                self.exit_code = 0 if ok else 1
            elif self._st_phase == 3 and t >= 9.5:
                spd = self.state.get('speed_kmh', 0.0)
                stopped = spd < 1.0
                print(f"🧪 SELFTEST(drive): speed after brake = {spd:.1f} km/h "
                      f"→ {'PASS (stopped)' if stopped else 'FAIL (still rolling)'}")
                self._set_held('brake', False)
                self.exit_code = 0 if (self.exit_code == 0 and stopped) else 1
                print("🧪 SELFTEST(drive): done")
                self._close_at = time.time() + 1.0
                self._st_phase = 9

    # --- Input handling ---

    def _on_key(self, event):
        down = event.type == pygame.KEYDOWN
        if self.mode == 'drive':
            held_map = {
                pygame.K_LEFT: 'steer_left', pygame.K_a: 'steer_left',
                pygame.K_RIGHT: 'steer_right', pygame.K_d: 'steer_right',
                pygame.K_UP: 'accelerate', pygame.K_w: 'accelerate',
                pygame.K_DOWN: 'brake', pygame.K_s: 'brake',
            }
            key = held_map.get(event.key)
            if key is not None:
                self._set_held(key, down)
                return
            if down and event.key == pygame.K_q:
                self._post('/control', {'blinker_left': True})
            elif down and event.key == pygame.K_e:
                self._post('/control', {'blinker_right': True})
            elif down and event.key == pygame.K_h:
                self._toggle_hazard()
            elif down and event.key == pygame.K_u:
                self._post('/control', {'uturn': True})
            elif down and event.key == pygame.K_f:
                self._toggle_driver_mode()
        else:  # test mode: type a test number, confirm with Enter
            if down and event.unicode.isdigit():
                self._numbuf = (self._numbuf + event.unicode)[-2:]
                self._last_digit_t = time.time()
                if len(self._numbuf) == 2:
                    val = int(self._numbuf)
                    self._numbuf = ""
                    if val in self.numbers:
                        self.request_jump(val)
            elif down and event.key == pygame.K_RETURN and self._numbuf:
                val = int(self._numbuf)
                self._numbuf = ""
                if val in self.numbers:
                    self.request_jump(val)
            elif down and event.key == pygame.K_BACKSPACE:
                self._numbuf = self._numbuf[:-1]

    # --- Drawing ---

    def _lamp(self, x: int, y: int, label: str, lit: bool, lit_color: tuple):
        r = pygame.Rect(x, y, 34, 26)
        fill = lit_color if lit else (58, 62, 70)
        pygame.draw.rect(self.screen, fill, r, border_radius=5)
        pygame.draw.rect(self.screen, lit_color if lit else PANEL_BORDER, r, 1,
                         border_radius=5)
        blit_centered(self.screen, label, 14, TEXT if lit else DIM, r.centerx, r.centery)

    def _warning(self, x: int, y: int, w: int, label: str, active: bool):
        r = pygame.Rect(x, y, w, 30)
        on = active and (self.frame // 12) % 2 == 0
        if on:
            pygame.draw.rect(self.screen, (150, 30, 30), r, border_radius=6)
            blit_centered(self.screen, label, 15, (255, 240, 240), r.centerx, r.centery)
        else:
            pygame.draw.rect(self.screen, PANEL, r, border_radius=6)
            pygame.draw.rect(self.screen, PANEL_BORDER, r, 1, border_radius=6)
            blit_centered(self.screen, label, 15, RED if active else DIM,
                          r.centerx, r.centery)

    def _draw_dashboard(self):
        p = pygame.Rect(12, 12, W - 24, 100)
        pygame.draw.rect(self.screen, PANEL, p, border_radius=8)
        pygame.draw.rect(self.screen, PANEL_BORDER, p, 1, border_radius=8)

        # Connection dot + label
        c = GREEN if self.connected else RED
        pygame.draw.circle(self.screen, c, (34, 36), 7)
        self.screen.blit(make_text("GAME " + ("ONLINE" if self.connected else "OFFLINE"),
                                   15, DIM), (50, 28))

        # Speed (big) + unit
        st = self.state
        kmh = int(st.get('speed_kmh', 0.0)) if st.get('has_car') else 0
        spd_color = TEXT if kmh <= 50 else (YELLOW if kmh <= 100 else RED)
        self.screen.blit(make_text(str(kmh), 46, spd_color), (120, 22))
        self.screen.blit(make_text("km/h", 16, DIM), (130 + 46 * len(str(kmh)) // 2, 58))

        # Driver mode + segment
        driver = st.get('driver') if st.get('has_car') else None
        mode_txt = {"BICYCLE": "AI (BICYCLE)", "FREE": "MANUAL (FREE)"}.get(driver, "NO CAR")
        mode_color = GREEN if driver == "BICYCLE" else (BLUE if driver == "FREE" else DIM)
        self.screen.blit(make_text(mode_txt, 18, mode_color), (300, 26))
        seg = st.get('segment')
        self.screen.blit(make_text(f"seg {seg}" if seg is not None else "seg -",
                                   18, DIM), (300, 54))

        # Lamps: accel / brake / blinkers / hazard
        lx = 470
        for label, lit, color in (
                ("A", bool(st.get('accelerating')), GREEN),
                ("B", bool(st.get('braking')), RED),
                ("L", bool(st.get('blinker_left')), ORANGE),
                ("R", bool(st.get('blinker_right')), ORANGE),
                ("H", bool(st.get('hazard')), ORANGE)):
            self._lamp(lx, 26, label, lit, color)
            lx += 44

        # Warnings
        self._warning(700, 26, 118, "OFF-ROAD", st.get('has_car') and not st.get('on_road', True))
        self._warning(700, 62, 118, "WRONG SIDE", bool(st.get('wrong_side')))

    def _draw_status(self):
        p = pygame.Rect(12, 122, W - 24, 64)
        pygame.draw.rect(self.screen, PANEL, p, border_radius=8)
        pygame.draw.rect(self.screen, PANEL_BORDER, p, 1, border_radius=8)
        if self.mode == 'test' and not self.connected:
            line1 = "Waiting for the game API… (start it with: python -m src.main --map basic --api)"
            line2 = ""
        else:
            line1, line2 = self.status_line1, self.status_line2
            if self.mode == 'test':
                pending = sum(1 for s in self.btn_state.values() if s == 'idle')
                running = sum(1 for s in self.btn_state.values() if s == 'running')
                tally = (f"✓ {self.tally['passed']} passed · ✗ {self.tally['failed']} failed · "
                         f"⏭ {self.tally['aborted']} aborted · "
                         f"▶ {running} running · … {pending} pending")
                line2 = (line2 + "    " + tally).strip() if line2 else tally
        self.screen.blit(make_text(line1, 19, TEXT), (28, 130))
        if line2:
            self.screen.blit(make_text(line2, 14, DIM), (28, 158))

    def _draw_test_panel(self):
        for n, r in self.scenario_rects.items():
            st = self.btn_state[n]
            if st == 'running':
                fill, border = (66, 60, 30), YELLOW
            elif st == 'passed':
                fill, border = (40, 92, 58), GREEN
            elif st == 'failed':
                fill, border = (108, 44, 44), RED
            else:
                fill, border = BTN_IDLE_FILL, BTN_IDLE_BORDER
            pygame.draw.rect(self.screen, fill, r, border_radius=8)
            pygame.draw.rect(self.screen, border, r, 2, border_radius=8)
            blit_centered(self.screen, str(n), 26, TEXT if st != 'idle' else DIM,
                          r.centerx, r.centery)
        hint = ("click a number — or type it and press Enter — to abort the current "
                "test and start that one")
        blit_centered(self.screen, hint, 14, DIM, W // 2, H - 26)

    def _draw_console_button(self, r: pygame.Rect, label: str, sub: str,
                             active: bool, active_color: tuple):
        fill = active_color if active else BTN_IDLE_FILL
        border = active_color if active else BTN_IDLE_BORDER
        pygame.draw.rect(self.screen, fill, r, border_radius=10)
        pygame.draw.rect(self.screen, border, r, 2, border_radius=10)
        blit_centered(self.screen, label, 22, TEXT, r.centerx, r.centery - 10)
        if sub:
            blit_centered(self.screen, sub, 12, DIM, r.centerx, r.centery + 18)

    def _draw_drive_panel(self):
        labels = {
            'steer_left': ("◀ STEER", "← / A"),
            'accelerate': ("GAS", "↑ / W"),
            'brake': ("BRAKE", "↓ / S"),
            'steer_right': ("STEER ▶", "→ / D"),
        }
        for key, r in self.drive_rects.items():
            self._draw_console_button(r, *labels[key], active=self.held[key],
                                      active_color=GREEN if key == 'accelerate' else BLUE)
        one_labels = {
            'blinker_left': ("BLINKER L", "Q"),
            'blinker_right': ("BLINKER R", "E"),
            'hazard': ("HAZARD", "H"),
            'uturn': ("U-TURN", "U"),
        }
        for key, r in self.one_shot_rects.items():
            active = (key == 'hazard' and self.hazard_on) or \
                     ((key == 'blinker_left' and self.state.get('blinker_left')) or
                      (key == 'blinker_right' and self.state.get('blinker_right')))
            self._draw_console_button(r, *one_labels[key], active=active,
                                      active_color=ORANGE)
        mode_txt = "F: switch to MANUAL (FREE)" if not self.free_mode \
            else "F: switch to AI (BICYCLE)"
        blit_centered(self.screen, DRIVE_HINT + "   ·   " + mode_txt, 14, DIM,
                      W // 2, H - 26)

    def _draw(self):
        self.frame += 1
        self.screen.fill(BG)
        self._draw_dashboard()
        self._draw_status()
        if self.mode == 'test':
            self._draw_test_panel()
            if self._numbuf:
                blit_centered(self.screen, f"jump to: {self._numbuf}⌫", 16, YELLOW,
                              W - 90, H // 2)
        else:
            self._draw_drive_panel()

    # --- Main loop ---

    def _on_sigint(self, signum, frame):
        # Ctrl-C: close the cockpit gracefully (same as clicking the window
        # close button) instead of dying with a traceback mid-frame.
        print("\n⏹  Ctrl-C - closing the cockpit...")
        try:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        except pygame.error:
            pass

    def run(self):
        signal.signal(signal.SIGINT, self._on_sigint)
        results = load_results()
        threading.Thread(target=self._poll_loop, daemon=True).start()
        if self.mode == 'test':
            threading.Thread(target=self._suite_worker, args=(results,),
                             daemon=True).start()
        else:
            self.status_line1 = "MANUAL DRIVE"
            self.status_line2 = DRIVE_HINT
        self._numbuf = ""
        self._last_digit_t = 0.0
        self._close_at: float | None = None
        self.t0 = time.time()

        while True:
            self.clock.tick(30)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return self._finish(closed_early=True)
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    if self.mode == 'test' and self._numbuf:
                        self._numbuf = ""
                    else:
                        return self._finish(closed_early=True)
                elif event.type in (pygame.KEYDOWN, pygame.KEYUP):
                    self._on_key(event)
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if self.mode == 'test':
                        for n, r in self.scenario_rects.items():
                            if r.collidepoint(event.pos):
                                self.request_jump(n)
                                break
                    else:
                        for key, r in self.drive_rects.items():
                            if r.collidepoint(event.pos):
                                self._set_held(key, True)
                                break
                        else:
                            for key, r in self.one_shot_rects.items():
                                if r.collidepoint(event.pos):
                                    if key == 'blinker_left':
                                        self._post('/control', {'blinker_left': True})
                                    elif key == 'blinker_right':
                                        self._post('/control', {'blinker_right': True})
                                    elif key == 'hazard':
                                        self._toggle_hazard()
                                    else:
                                        self._post('/control', {'uturn': True})
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    if self.mode == 'drive':
                        for key in self.drive_rects:
                            self._set_held(key, False)

            # Expire a half-typed jump number.
            if self.mode == 'test' and self._numbuf and \
                    time.time() - self._last_digit_t > 1.5:
                self._numbuf = ""

            self._process_events()
            self._selftest(time.time())
            self._draw()
            pygame.display.flip()

            if self._close_at is not None and time.time() >= self._close_at:
                return self._finish(closed_early=False)

    def _finish(self, closed_early: bool) -> int:
        # Release anything still held so the car doesn't keep driving.
        for key in list(self.held):
            if self.held[key]:
                self._set_held(key, False)
        self._exit_evt.set()
        if self.mode == 'test' and closed_early and not self.suite_done:
            print("\n⏹  Controller closed mid-suite - completed results are saved; "
                  "the game keeps running.")
        pygame.quit()
        return self.exit_code


def main():
    ap = argparse.ArgumentParser(
        description="Car controller cockpit (dashboard + test buttons / driving console)")
    ap.add_argument('--url', default=os.environ.get('CAR_API_URL', API_URL),
                    help=f"game REST API base URL (default {API_URL})")
    ap.add_argument('--mode', choices=['test', 'drive'], default='test',
                    help="test: run the e2e suite with numbered jump buttons "
                         "(default); drive: manual driving console")
    ap.add_argument('--tests', default=None,
                    help="scenario selection for test mode (e.g. '1-3' or 'y_from_stem')")
    ap.add_argument('--selftest', nargs='?', const=2, type=int, default=None, metavar='N',
                    help="scripted smoke test: in test mode simulates a click on "
                         f"button N (default 2) after 6 s; in drive mode drives the "
                         "car with gas/brake and verifies it moves and stops")
    args = ap.parse_args()

    controller = Controller(args)
    code = controller.run()
    sys.exit(code)


if __name__ == '__main__':
    main()
