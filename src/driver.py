# Driver Classes
# Abstract base class and implementations for controlling cars

from __future__ import annotations
from abc import ABC, abstractmethod
import math
import pygame


class Driver(ABC):
    """Abstract base class for car controllers."""
    
    @abstractmethod
    def get_control(self, car, network, dt, keys) -> dict:
        """Return control inputs for the car.
        
        Returns dict with:
            accelerate: bool
            brake: bool
            steer_left: bool
            steer_right: bool
            blinker_left: bool (for AI)
            blinker_right: bool (for AI)
        """
        pass
    
    @abstractmethod
    def get_name(self) -> str:
        """Return driver type name for display."""
        pass


class KeyboardDriver(Driver):
    """Human player controlling via keyboard (FREE mode).

    Keys: WASD/arrows drive, Q/E blinkers (held = on, like holding the
    stalk). Gears work like a real car: while moving forward, S brakes to
    a stop - pressing S AGAIN engages reverse. While reversing, W brakes
    to a stop - pressing W AGAIN drives forward. Holding a brake key
    through zero never shifts gears; a fresh press is required.
    """
    
    def __init__(self):
        self.name = "KEYBOARD"
        self.blinker_left = False
        self.blinker_right = False
        # Previous frame's W/S state - used to detect FRESH key-downs,
        # which are what engage a gear at a standstill.
        self._last_w = False
        self._last_s = False
    
    def get_control(self, car, network, dt, keys) -> dict:
        """Manual steering control."""
        # Momentary blinkers (held = on). Stored on the driver too - the
        # renderer and HUD read driver.blinker_left/right for the lights.
        self.blinker_left = bool(keys[pygame.K_q])
        self.blinker_right = bool(keys[pygame.K_e])

        w = keys[pygame.K_UP] or keys[pygame.K_w]
        s = keys[pygame.K_DOWN] or keys[pygame.K_s]
        control = {
            'accelerate': w,
            'brake': s,
            # Fresh key-down edges (see Car._update_free_mode): a new S
            # press at a standstill engages reverse, a new W press drives
            # forward. Holding the brake through zero must NOT shift.
            'accelerate_pressed': w and not self._last_w,
            'brake_pressed': s and not self._last_s,
            'steer_left': keys[pygame.K_LEFT] or keys[pygame.K_a],
            'steer_right': keys[pygame.K_RIGHT] or keys[pygame.K_d],
            'blinker_left': self.blinker_left,
            'blinker_right': self.blinker_right,
        }
        self._last_w = w
        self._last_s = s
        return control
    
    def get_name(self) -> str:
        return "FREE"


class BicycleDriver(Driver):
    """Autonomous driver for BICYCLE mode.

    Provides high-level intent (accelerate / brake / which way to turn,
    from the keyboard or the REST API). The car executes it with the
    kinematic bicycle model (src/bicycle_nav.py).
    """

    # Destination parking (spec §1): the nav evaluates the stateless brake
    # & park plan every tick from (distance to stop point, speed) and owns
    # the deceleration itself - no trigger distance or brake latch here.
    # This driver only mirrors the indicator from the plan's phase
    # (nav.park_phase): on from the lead phase until the car is stopped.

    def __init__(self):
        self.name = "BICYCLE"
        self.pending_turn = None  # "left", "right", or None
        self.blinker_left = False
        self.blinker_right = False
        self._last_left = False
        self._last_right = False
        # One-shot U-turn (Wenden) request, set by the 'u' key or the REST
        # API; consumed by BicycleNav on the next frame.
        self.uturn_requested = False
        self._uturn_was_active = False
        # Remember whether we were just pulling out of the kerb, so the
        # pull-out blinker can be switched off exactly once - see the
        # elif chain in get_control (a bare `_pull_out_frames <= 0` test
        # there was TRUE in all normal driving and killed every manually
        # signaled LEFT blinker within one frame).
        self._was_pulling_out = False
        # Which indicator WE set for an active lane change (merge before
        # parking) - None when no lane-change signal of ours is on. Used
        # to switch it off exactly once when the merge settles, without
        # touching user-signaled or turn indicators.
        self._lane_change_side = None
        # Hazard lights (Warnblinkanlage): all four corner blinkers flash.
        # Turned on automatically when the car recognises it cannot
        # continue (e.g. U-turn stall) or manually via the REST API.
        # They stay on for at least HAZARD_MIN_DISPLAY_S so a stuck state
        # is visible, not just a crash.
        self.hazard = False
        self.hazard_reason = ""
        self.hazard_on_at: float | None = None

    HAZARD_MIN_DISPLAY_S = 5.0

    # NOTE: the old steering-cam auto-off (STEER_IN_DEG / BACK_CENTRE_DEG,
    # clearing pending_turn once the wheel steered in and back to centre)
    # is gone: it cleared the intent on any steering DIP, which broke
    # multi-decision-point maneuvers - measured on the roundabout, it fired
    # mid-approach to the exit corner (pure-pursuit dips below 6 deg while
    # the extreme had already hit 38), so the route reverted to 'straight'
    # and the car skipped the exit. pending_turn is now cleared by the nav
    # when the car actually PASSES the junction the signal was for
    # (BicycleNav._turn_signal_target); _clear_turn_signal does both the
    # light and the intent.

    def set_hazard(self, on: bool, reason: str = "") -> None:
        """Turn the hazard lights on/off (with logging).

        Off is ignored while the minimum display time is still running -
        a stuck state must stay visible for at least 5 seconds.
        """
        import time
        if on and not self.hazard:
            self.hazard = True
            self.hazard_reason = reason or "unspecified"
            self.hazard_on_at = time.time()
            print(f"\n🚨 HAZARD LIGHTS ON - {self.hazard_reason}\n")
        elif not on and self.hazard:
            elapsed = (time.time() - self.hazard_on_at
                       if self.hazard_on_at is not None else 0.0)
            if elapsed < self.HAZARD_MIN_DISPLAY_S:
                print(f"🚨 Hazard lights stay on: minimum display time "
                      f"({self.HAZARD_MIN_DISPLAY_S:.0f} s) still running "
                      f"({elapsed:.1f} s elapsed)")
                return
            self.hazard = False
            self.hazard_reason = ""
            self.hazard_on_at = None
            print("\n🚨 Hazard lights OFF\n")

    def get_control(self, car, network, dt, keys) -> dict:
        """Automatic road following with blinkers."""
        # Update blinkers based on A/D keys
        left = keys[pygame.K_LEFT] or keys[pygame.K_a]
        right = keys[pygame.K_RIGHT] or keys[pygame.K_d]

        # Toggle blinkers
        if left and not self._last_left:
            self.blinker_left = not self.blinker_left
            if self.blinker_left:
                self.blinker_right = False
                self.pending_turn = "left"
            else:
                self.pending_turn = None
        if right and not self._last_right:
            self.blinker_right = not self.blinker_right
            if self.blinker_right:
                self.blinker_left = False
                self.pending_turn = "right"
            else:
                self.pending_turn = None

        self._last_left = left
        self._last_right = right

        # W/S for speed control (manual override)
        accel = keys[pygame.K_UP] or keys[pygame.K_w]
        brake = keys[pygame.K_DOWN] or keys[pygame.K_s]

        # Destination parking (spec §1): mirror the indicator from the
        # nav's brake & park plan phase. The plan owns the deceleration;
        # nothing here latches a brake.
        nav = car.bicycle_nav
        uturn_now = nav is not None and getattr(nav, 'uturn_active', False)
        if self._uturn_was_active and not uturn_now:
            # Maneuver just finished: indicator off (spec §5: "Blinker aus").
            self.blinker_left = False
        self._uturn_was_active = uturn_now
        if uturn_now:
            # U-turn in progress: left blinker on for the WHOLE maneuver,
            # throttle on, and the parking/pull-out logic below is suspended
            # (the nav's signed speed profile drives everything).
            self.blinker_left = True
            self.blinker_right = False
            self.pending_turn = None
            return {
                'accelerate': True,
                'brake': False,
                'steer_left': False,
                'steer_right': False,
                'blinker_left': True,
                'blinker_right': False,
            }
        dist_dest = nav.distance_to_destination() if nav is not None else None
        s_pos = nav._s if nav is not None else 0.0
        pulling_out = nav is not None and nav._pull_out_frames > 0
        park_phase = getattr(nav, 'park_phase', 'none') \
            if nav is not None else 'none'
        parking = park_phase in ('lead', 'decel', 'swerve', 'final',
                                 'reverse')
        # An explicit turn signal for a REAL branch at the next junction
        # must survive the parking block: on short streets ending in a
        # cul-de-sac, the plan can go active before the car reaches the
        # junction, and wiping pending_turn here used to rebuild the route
        # straight through - the car blinked for a turn it then never made.
        # After the turn executes the signal clears (auto-off) and parking
        # resumes as usual.
        parking_blocked_by_turn = self._signal_would_turn(car, network)
        # Lane change before parking (docs §1 variant, user rule): ALWAYS
        # signal BEFORE changing lanes. The nav owns the timing - on from
        # MERGE_SIGNAL_AHEAD_M before the merge zone starts, off once the
        # car has settled onto the new line. While active it wins over the
        # parking/pull-out signals; when it drops we switch off only the
        # indicator WE set (a user-signaled or turn signal must survive).
        lane_change = getattr(nav, 'lane_change_signal', None) \
            if nav is not None else None
        if lane_change is None and getattr(self, '_lane_change_side', None):
            if self._lane_change_side == 'right':
                self.blinker_right = False
            else:
                self.blinker_left = False
            self._lane_change_side = None
        if lane_change == 'right':
            self.blinker_right = True
            self.blinker_left = False
            self.pending_turn = None
            self._lane_change_side = 'right'
        elif lane_change == 'left':
            self.blinker_left = True
            self.blinker_right = False
            self.pending_turn = None
            self._lane_change_side = 'left'
        elif parking and not parking_blocked_by_turn:
            self.blinker_right = True
            self.blinker_left = False
            self.pending_turn = None
            accel = False
        elif pulling_out:
            # Pulling out from the right edge: signal LEFT (into lane)
            self.blinker_left = True
            self.blinker_right = False
            self.pending_turn = None
            accel = True
            brake = False
        elif park_phase == 'stopped':
            # Stopped at the destination: switch the parking blinker off
            # (spec §1: "Blinker aus").
            self.blinker_right = False
        elif self.blinker_left and nav is not None and \
                self._was_pulling_out and nav._pull_out_frames <= 0:
            # Pull-out just finished: switch the pull-out blinker off.
            # (Only after an actual pull-out - a user-signaled left turn
            # must stay on until the turn is executed.)
            self.blinker_left = False

        self._was_pulling_out = pulling_out

        return {
            'accelerate': accel,
            'brake': brake,
            'steer_left': False,  # AI controls steering via road following
            'steer_right': False,
            'blinker_left': self.blinker_left,
            'blinker_right': self.blinker_right,
        }

    def _signal_would_turn(self, car, network) -> bool:
        """True if the currently pending signal would actually take a
        branch at the NEXT junction (a real turn exists there).

        Computed here - not from the nav's current route - because the
        parking block below runs BEFORE the nav rebuilds for the new
        signal: reading a post-rebuild flag would race and wipe a signal
        set this very frame. Same test the nav applies when building:
        choose_next_segment falls back to the straight continuation when
        no branch matches, so "chosen != straight" is the turn test.
        The naive test used to be "chosen != straight": but at a plain
        T-junction the stem has no straight-ahead ROAD at all, so
        choose_next_segment's straight fallback picks whichever branch is
        geometrically closest to straight ahead - which can be the very
        same segment 'right' or 'left' resolves to. There the test read
        "not a real turn" for a turn that is the ONLY way to leave the stem,
        wiped the signal, and rebuilt the route with turn=None (harmless
        here only because 'straight' happened to choose the same branch -
        on a Y it would not). The real test is whether a road called
        'straight' exists here at all: if it does not, ANY signalled branch
        is a genuine, deliberate turn.
        """
        if self.pending_turn not in ('left', 'right'):
            return False
        seg = network.segments[car.seg_idx]
        jnode = seg.end_node if car.forward else seg.start_node
        if network.node_degree.get(jnode, 0) < 3:
            return False        # no junction ahead at all
        chosen = network.choose_next_segment(car.seg_idx, jnode,
                                             self.pending_turn)
        if chosen is None:
            return False
        connected = network.get_connected_segments(jnode)
        has_straight = any(
            abs(network.get_exit_angle(car.seg_idx, idx)) < 30.0
            for idx in connected if idx != car.seg_idx)
        if not has_straight:
            return True         # no straight road here: any turn is real
        straight = network.choose_next_segment(car.seg_idx, jnode,
                                               "straight")
        return chosen != straight

    def signal_turn(self, direction: str):
        """Arm a turn signal (used by the REST API one-shot commands and
        by BicycleNav's roundabout-exit re-arm). The nav clears it again
        when the car passes the junction the signal was for."""
        self.pending_turn = direction
        if direction == 'left':
            self.blinker_left = True
            self.blinker_right = False
        else:
            self.blinker_right = True
            self.blinker_left = False

    def _clear_turn_signal(self):
        self.blinker_left = False
        self.blinker_right = False
        self.pending_turn = None

    def get_name(self) -> str:
        return "BICYCLE"
