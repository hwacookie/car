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


class AIDriver(Driver):
    """Autonomous driver that follows roads (RAILS mode)."""
    
    def __init__(self):
        self.name = "AI"
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
        
        # Determine if we should brake automatically
        should_brake = self._should_brake_for_turn(car, network, dt)
        
        # W/S for speed control (manual override)
        accel = keys[pygame.K_UP] or keys[pygame.K_w]
        brake = keys[pygame.K_DOWN] or keys[pygame.K_s]
        
        return {
            'accelerate': accel,
            'brake': brake or should_brake,  # Automatic + manual braking
            'steer_left': False,  # AI controls steering via road following
            'steer_right': False,
            'blinker_left': self.blinker_left,
            'blinker_right': self.blinker_right,
        }
    
    def _should_brake_for_turn(self, car, network, dt) -> bool:
        """Determine if automatic braking is needed for upcoming turn."""
        if not self.pending_turn or car.speed < 1.0:
            return False
        
        seg = network.segments[car.seg_idx]
        node = seg.end_node if car.forward else seg.start_node
        
        # Only brake at real junctions with right-of-way conflict
        node_deg = network.node_degree.get(node, 2)
        if node_deg < 3:
            return False
        
        if not network.has_right_of_way_conflict(car.seg_idx, node):
            return False
        
        next_seg = network.choose_next_segment(car.seg_idx, node, self.pending_turn)
        if next_seg is None or next_seg == car.seg_idx:
            return False
        
        turn_angle = abs(network.get_exit_angle(car.seg_idx, next_seg))
        
        # Determine safe speed
        if turn_angle > 90:
            safe_speed = 25 / 3.6
        elif turn_angle > 60:
            safe_speed = 40 / 3.6
        elif turn_angle > 30:
            safe_speed = 55 / 3.6
        else:
            return False
        
        if car.speed <= safe_speed:
            return False
        
        # Calculate if we need to brake now
        if car.forward:
            remaining_distance = seg.length * (1.0 - car.progress)
        else:
            remaining_distance = seg.length * car.progress
        
        from . import config
        braking_distance = (car.speed**2 - safe_speed**2) / (2 * config.CAR_BRAKING)
        safety_margin = 15.0
        
        return remaining_distance <= braking_distance + safety_margin
    
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
        return "RAILS"
