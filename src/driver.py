# Driver Classes
# Abstract base class and implementations for controlling cars

from __future__ import annotations
from abc import ABC, abstractmethod
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
    """Human player controlling via keyboard (FREE mode)."""
    
    def __init__(self):
        self.name = "KEYBOARD"
    
    def get_control(self, car, network, dt, keys) -> dict:
        """Manual steering control."""
        return {
            'accelerate': keys[pygame.K_UP] or keys[pygame.K_w],
            'brake': keys[pygame.K_DOWN] or keys[pygame.K_s],
            'steer_left': keys[pygame.K_LEFT] or keys[pygame.K_a],
            'steer_right': keys[pygame.K_RIGHT] or keys[pygame.K_d],
            'blinker_left': False,
            'blinker_right': False,
        }
    
    def get_name(self) -> str:
        return "FREE"


class BicycleDriver(Driver):
    """Autonomous driver for BICYCLE mode.

    Provides high-level intent (accelerate / brake / which way to turn,
    from the keyboard or the REST API). The car executes it with the
    kinematic bicycle model (src/bicycle_nav.py).
    """

    # When approaching the destination (a dead end) within this distance,
    # signal right and brake - like a real driver pulling over to the right
    # edge of the road.
    PARK_DISTANCE_M = 50.0

    def __init__(self):
        self.name = "BICYCLE"
        self.pending_turn = None  # "left", "right", or None
        self.blinker_left = False
        self.blinker_right = False
        self._last_left = False
        self._last_right = False

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

        # Parking: when approaching the destination (a dead end) within
        # PARK_DISTANCE_M, signal right and brake - like a real driver
        # pulling over to the right edge. The nav interprets brake + right
        # blinker (route ending at a dead end) as "pull over to the right
        # edge."
        nav = car.bicycle_nav
        dist_dest = nav.distance_to_destination() if nav is not None else None
        s_pos = nav._s if nav is not None else 0.0
        pulling_out = nav is not None and nav._pull_out_frames > 0
        parking = dist_dest is not None and dist_dest <= self.PARK_DISTANCE_M
        if parking:
            self.blinker_right = True
            self.blinker_left = False
            self.pending_turn = None
            brake = True
            accel = False
        elif pulling_out:
            # Pulling out from the right edge: signal LEFT (into lane)
            self.blinker_left = True
            self.blinker_right = False
            self.pending_turn = None
            accel = True
            brake = False
        elif self.blinker_right and car.speed < 0.1 and \
                dist_dest is not None and dist_dest < 1.0:
            # Stopped at the destination: switch the parking blinker off.
            self.blinker_right = False
        elif self.blinker_left and nav is not None and nav._pull_out_frames <= 0:
            # Finished pulling out into lane: switch left blinker off.
            self.blinker_left = False

        return {
            'accelerate': accel,
            'brake': brake,
            'steer_left': False,  # AI controls steering via road following
            'steer_right': False,
            'blinker_left': self.blinker_left,
            'blinker_right': self.blinker_right,
        }

    def clear_blinker_if_turned(self, car, network, from_seg: int, to_seg: int):
        """Clear blinker if actually turned in the signaled direction."""
        if from_seg == to_seg:
            return

        turn_angle = network.get_exit_angle(from_seg, to_seg)

        if self.blinker_left and turn_angle < -30:
            self.blinker_left = False
            self.pending_turn = None
        elif self.blinker_right and turn_angle > 30:
            self.blinker_right = False
            self.pending_turn = None

    def get_name(self) -> str:
        return "BICYCLE"
