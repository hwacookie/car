# Car Class - Pure Physics and State
# No input handling - controlled by Driver classes

from __future__ import annotations

import math
import pygame

from . import config
from .turning_system import TurningSystem, TurnPlan


class Car:
    """A car with physics, state, and rendering. Controlled by a Driver."""
    
    # Class variable: cached sprite images
    _sprite_cache = {}
    
    def __init__(self, x: float, y: float, heading: float, seg_idx: int, driver=None):
        """x, y in world pixels. heading in degrees (0 = up/north)."""
        # Position and orientation
        self.x = x
        self.y = y
        self.heading = heading
        
        # Speed
        self.speed = 0.0  # m/s
        self.target_speed = 0.0
        
        # Road following state (for RAILS mode)
        self.seg_idx = seg_idx
        self.progress = 0.5  # 0.0 to 1.0 along segment
        self.forward = True  # Direction on segment
        
        # Driver controlling this car
        self.driver = driver
        
        # Bicycle-model navigation (BICYCLE mode; created lazily on first
        # use - see update()). None in RAILS/FREE mode.
        self.bicycle_nav = None
        
        # Turning system
        self.turning_system = TurningSystem(max_lateral_accel=5.0)
        self.active_turn: TurnPlan | None = None
        # 1.0 = full right-lane offset (normal driving), 0.0 = centerline
        # (at the exact tangent point of an upcoming turn). Blended down
        # smoothly as we approach a turn's tangent point, so the car
        # gradually swerves toward - and, only if the corner geometry
        # genuinely requires it, across - the centerline in preparation,
        # rather than snapping from lane position to centerline instantly.
        self._lane_offset_factor: float = 1.0
        self._recovery_start_factor: float = 1.0
        # Where (segment + progress) the recovery blend above started
        # from - see _advance_active_turn() / the recovery-blend branch of
        # _update_position_rails() for why this must be measured from the
        # actual hand-off point, not from the segment's start.
        self._recovery_start_seg_idx: int | None = None
        self._recovery_start_progress: float = 0.0
        
        # The turn plan for an upcoming junction, decided ONCE (fixed
        # target speed, fixed geometry) as soon as we know which way we're
        # turning - like a human sizing up a corner well in advance rather
        # than reactively recalculating the required radius every frame
        # from a still-changing current speed (see planning discussion in
        # conversation history / SPEC.md). Braking is then just "do we
        # need to slow down to reach this FIXED target speed by this
        # FIXED point" - a single, stable calculation per frame.
        self.planned_turn: TurnPlan | None = None
        self.planned_turn_key: tuple | None = None  # (from_seg_idx, to_seg_idx, junction_node)
        self._warned_no_arc_for_node: str | None = None
        # Set (by _get_or_create_planned_turn) when the driver's signaled
        # next segment at the upcoming junction exists but NO arc radius
        # fits for it (not even at the mechanical minimum) - i.e. the
        # blinker was set too late to make that turn. _handle_segment_end
        # then slides the car past the junction on the straight
        # continuation (or crashes into the dead-end obstacle) instead of
        # force-entering the unreachable segment - see
        # docs/TURN_REWORK_PLAN.md (blinkers mean "next REACHABLE
        # decision point", never "force this one").
        self._pending_no_arc_next_seg: int | None = None
        
        # Crash state (dead-end T, see _crash_into_obstacle): frames of
        # impact deceleration still to apply.
        self._crash_frames_left = 0
        self._crash_decel = 0.0
        
        # Visual state
        self._braking = False
        self._accelerating = False
        
        # Smooth heading transition
        self._heading_transition = False
        self._heading_transition_progress = 0.0
        self._heading_start = 0.0
        self._heading_target = 0.0
        self._heading_transition_duration = 0.3
        # Rate-limited transition state (see
        # _begin_heading_transition_for_segment): degrees of rotation
        # still to perform (signed), time elapsed, and the radius (m)
        # the rotation rate is capped to keep (v / omega >= r).
        self._heading_transition_remaining = 0.0
        self._heading_transition_elapsed = 0.0
        self._heading_transition_radius_m = 4.0
        
        # Debug trail
        self.trail = []
        self._trail_timer = 0.0
        self.trail_enabled = False
    
    # --- Main Update ---
    
    def update(self, dt: float, network, control_input: dict):
        """Update car physics based on control input from driver.
        
        control_input dict:
            accelerate: bool
            brake: bool
            steer_left: bool (FREE mode only)
            steer_right: bool (FREE mode only)
            blinker_left: bool (RAILS mode, from driver state)
            blinker_right: bool (RAILS mode, from driver state)
        """
        old_x, old_y = self.x, self.y
        
        # Update visual state
        self._braking = control_input.get('brake', False)
        self._accelerating = control_input.get('accelerate', False)
        
        # Choose update mode based on driver type
        name = self.driver.get_name() if self.driver else "FREE"
        if name == "BICYCLE":
            if self.bicycle_nav is None:
                from .bicycle_nav import BicycleNav
                self.bicycle_nav = BicycleNav(self, network)
            self.bicycle_nav.update(dt, control_input)
        elif name == "RAILS":
            self._update_rails_mode(dt, network, control_input)
        else:
            self._update_free_mode(dt, control_input)
        
        # Update trail
        if self.trail_enabled:
            self._trail_timer += dt
            if self._trail_timer >= 0.1:
                self._trail_timer = 0.0
                self.trail.append((self.x, self.y, self.heading))
                if len(self.trail) > 500:
                    self.trail.pop(0)
    
    # --- FREE Mode Physics ---
    
    def _update_free_mode(self, dt: float, control_input: dict):
        """Manual steering mode with cruise control."""
        accel = control_input.get('accelerate', False)
        brake = control_input.get('brake', False)
        steer_left = control_input.get('steer_left', False)
        steer_right = control_input.get('steer_right', False)
        
        # Speed control
        if accel:
            self.speed += config.CAR_ACCELERATION * dt
        elif brake:
            self.speed -= config.CAR_BRAKING * dt
        
        self.speed = max(0, min(config.CAR_SPEED, self.speed))
        self.target_speed = self.speed
        
        # Steering (only when moving)
        if self.speed > 0:
            turn_factor = max(0.3, 1.0 - self.speed / config.CAR_SPEED * 0.7)
            turn_rate = config.CAR_TURN_SPEED * turn_factor * dt
            if steer_left:
                self.heading -= turn_rate
            if steer_right:
                self.heading += turn_rate
            self.heading = self.heading % 360
        
        # Movement
        rad = math.radians(self.heading)
        dx = math.sin(rad) * self.speed * dt * config.PIXELS_PER_METER
        dy = math.cos(rad) * self.speed * dt * config.PIXELS_PER_METER
        self.x += dx
        self.y += dy
    
    # --- RAILS Mode Physics ---
    
    def _update_rails_mode(self, dt: float, network, control_input: dict):
        """Automatic road following mode with physics-based turning."""
        accel = control_input.get('accelerate', False)
        brake = control_input.get('brake', False)
        
        # While actively executing a turn's arc, the geometry was built
        # for a FIXED speed (turn_plan.target_speed_mps) - a driver holding
        # the accelerator through a corner shouldn't be able to speed up
        # mid-turn and exceed what that fixed-radius arc was planned for
        # (that's exactly what "accelerate whenever possible, except while
        # actively slowing for a turn" means: once we're IN the turn,
        # further acceleration is deferred until it's over). Braking is
        # still allowed to reduce speed further at any time.
        speed_cap = self.active_turn.target_speed_mps if self.active_turn else config.CAR_SPEED
        
        # Crash into obstacle (dead-end T): the impact drains speed
        # directly, overriding any driver input (see _crash_into_obstacle).
        if self._crash_frames_left > 0:
            self.speed = max(0.0, self.speed - self._crash_decel * dt)
            self.target_speed = 0.0
            self._braking = True
            if self.speed <= 0.0:
                self._crash_frames_left = 0
            else:
                self._crash_frames_left -= 1
        else:
            # Speed control
            if accel:
                self.target_speed = min(self.target_speed + config.CAR_ACCELERATION * dt, speed_cap)
            elif brake:
                self.target_speed = max(self.target_speed - config.CAR_BRAKING * dt, 0)
                # Direct braking
                self.speed -= config.CAR_BRAKING * dt
            
            # Approach target speed
            if self.speed < self.target_speed:
                self.speed += config.CAR_ACCELERATION * dt
            elif self.speed > self.target_speed:
                self.speed -= config.CAR_BRAKING * dt * 0.3
            
            self.speed = max(0, min(speed_cap, self.speed))
        
        # Move along road
        if self.speed > 0:
            # Check if we're currently executing a turn
            if self.active_turn:
                self._execute_active_turn(dt, network)
            else:
                # Normal segment following
                self._update_position_rails(dt, network)
        
        # Advance any smooth heading transition started by a segment-end
        # hand-off (see _begin_heading_transition_for_segment). Runs
        # AFTER the position update, which suppresses its own heading
        # write while a transition is active (see
        # _apply_plain_segment_position).
        self._update_heading_transition(dt)
    
    def _execute_active_turn(self, dt: float, network):
        """Execute current turn using circular arc (this frame's full distance)."""
        self._advance_active_turn(self.speed * dt, network)
    
    def _advance_active_turn(self, distance_m: float, network):
        """Advance the active turn by an explicit distance (meters).
        
        Split out from _execute_active_turn so the initial hand-off from
        plain segment-following onto the arc can move PART of a frame's
        distance on the straight segment (up to the exact tangent point)
        and the REMAINDER along the arc, all within the same frame - no
        discrete jump, see _update_position_rails().
        """
        # Move along arc
        x, y, heading, new_progress = self.turning_system.execute_turn(
            self.active_turn, distance_m
        )
        
        self.x = x
        self.y = y
        self.heading = heading
        self.active_turn.progress = new_progress
        
        # Check if turn complete
        if new_progress >= 1.0:
            # Transition to next segment
            self.seg_idx = self.active_turn.to_seg_idx
            to_seg = network.segments[self.seg_idx]
            
            # Set direction on new segment
            self.forward = (to_seg.start_node == self.active_turn.junction_node)
            
            # IMPORTANT: The arc's endpoint is NOT at the junction node -
            # it's to_tangent_offset_m PAST it (that's the whole point of
            # the tangent-fillet construction: the arc leaves the curve
            # already partway along the new road). Setting progress to
            # 0.0/1.0 ("at the junction") and snapping position back to the
            # node was causing a visible backward jump every time a turn
            # completed. Instead, set progress to match EXACTLY where the
            # arc's own (x, y) already is - self.x/self.y from the arc
            # execution above are left untouched here, and progress is
            # derived to be consistent with them, so there is no jump at
            # all, just a continuous handoff to plain segment-following.
            # Uses to_tangent_offset_m (measured from the TRUE junction,
            # consistent with plain segment-following's own
            # parametrization) rather than the geometric tangent_distance_m
            # (measured from the lane-offset-shifted effective junction,
            # which can differ by the offset's along-track component).
            frac_along = self.active_turn.to_tangent_offset_m / to_seg.length if to_seg.length > 0 else 0.0
            frac_along = max(0.0, min(1.0, frac_along))
            self.progress = frac_along if self.forward else 1.0 - frac_along
            
            # Notify driver that turn completed
            if self.driver and hasattr(self.driver, 'clear_blinker_if_turned'):
                self.driver.clear_blinker_if_turned(
                    self, network, 
                    self.active_turn.from_seg_idx, 
                    self.active_turn.to_seg_idx
                )
            
            # Preserve the arc's ABSOLUTE lateral position across the
            # hand-off. _lane_offset_factor is a FRACTION of the CURRENT
            # segment's width/4, so the same factor number means a
            # different absolute offset on segments of different width:
            # a turn from a 7 m road onto a 3.5 m road ends the arc
            # 1.75 m from the new road's centerline, which there is
            # factor 2.0 (the road edge - still on the paved surface,
            # the arc was validated against it). Keeping the old factor
            # (a fraction of the WIDE road's quarter width) would
            # re-render the car at 0.87 m from centerline on the narrow
            # road - a ~0.9 m lateral teleport, which the watchdog
            # rightly killed the game for. Derive the factor from the
            # arc's own absolute lane offset instead: by construction
            # the arc's endpoint lies exactly lane_offset_m to the right
            # of the TO road's centerline, so the matching factor is
            # lane_offset_m / (to_seg.width / 4). It can exceed 1.0
            # (road edge on a narrower exit road) or be negative
            # (opposing-lane plans); the recovery blend below eases it
            # back toward 1.0 (own-lane center) over the next 20 m, so
            # the car naturally settles from wherever the arc left it.
            if not to_seg.oneway and to_seg.width > 0:
                self._lane_offset_factor = (
                    self.active_turn.lane_offset_m / (to_seg.width / 4.0)
                )
            
            # Remember the lane-offset factor the car is actually at
            # after the hand-off (see above) - the recovery blend on the
            # new segment eases FROM this value toward 1.0, instead of
            # assuming it always starts from 0 or from a stale fraction
            # of a different road's width.
            self._recovery_start_factor = self._lane_offset_factor
            # Also remember WHERE (segment + progress) this hand-off
            # happened - the tangent-fillet arc can end well past the
            # segment's own start (to_tangent_offset_m can easily exceed
            # the 20m recovery blend distance below, as it does on tight
            # corners with a long tangent distance), so the blend must be
            # measured from THIS point, not from the segment's start.
            # Using "distance from segment start" there previously made
            # the blend already mathematically "complete" (capped to 1.0)
            # by the very next frame whenever to_tangent_offset_m alone
            # exceeded the blend distance - snapping the lane factor
            # straight from its hand-off value to 1.0 in a single frame
            # instead of easing there, which is exactly the kind of
            # sudden sideways jump the teleportation watchdog is (rightly)
            # designed to catch.
            self._recovery_start_seg_idx = self.seg_idx
            self._recovery_start_progress = self.progress
            
            # Clear active turn
            self.active_turn = None
    
    def _update_position_rails(self, dt: float, network):
        """Move along current segment, checking for upcoming turns.
        
        PLAN-ONCE MODEL: the moment we know which way we're turning at the
        upcoming junction, we decide (once, fixed) a target speed and the
        exact arc geometry for that speed - like a human sizing up a
        corner well in advance, rather than reactively recalculating the
        required radius every frame from whatever speed we currently
        happen to be going. From there, each frame just asks one simple,
        stable question: "do I need to brake NOW to reach that fixed
        target speed by that fixed point?" Gentle turns naturally need no
        braking at all if the target speed is already at/above cruise.
        """
        if self.seg_idx >= len(network.segments):
            return
        
        seg = network.segments[self.seg_idx]
        distance_m = self.speed * dt
        junction_node = seg.end_node if self.forward else seg.start_node
        
        if self.forward:
            remaining_to_junction_m = seg.length * (1.0 - self.progress)
        else:
            remaining_to_junction_m = seg.length * self.progress
        
        # Get (or create) the ONE-TIME plan for whichever way the driver
        # intends to go at this junction. Cheap to call every frame: only
        # actually (re)plans when the intended direction changes.
        turn_plan = self._get_or_create_planned_turn(network, junction_node)
        
        if turn_plan and not self.active_turn:
            # Uses from_tangent_offset_m (measured from the TRUE junction)
            # rather than tangent_distance_m (measured from the lane-
            # offset-shifted effective junction) - see TurnPlan docstring.
            distance_to_tangent_m = remaining_to_junction_m - turn_plan.from_tangent_offset_m
            
            # Brake NOW (full force, directly - not the gentler cruise
            # blend) if that's what it takes to reach the FIXED target
            # speed by the FIXED tangent point. A real driver brakes
            # firmly and deliberately for a corner, not via gentle cruise
            # control, so this uses config.CAR_BRAKING at full strength to
            # match the distance calculation below exactly.
            if self.speed > turn_plan.target_speed_mps:
                braking_distance_m = (
                    (self.speed ** 2 - turn_plan.target_speed_mps ** 2) / (2 * config.CAR_BRAKING)
                )
                safety_margin_m = 5.0
                if distance_to_tangent_m <= braking_distance_m + safety_margin_m:
                    self.speed = max(turn_plan.target_speed_mps, self.speed - config.CAR_BRAKING * dt)
                    self.target_speed = min(self.target_speed, turn_plan.target_speed_mps)
                    self._braking = True
            
            # Smoothly blend from full right-lane offset (1.0) toward
            # whichever fraction the PLAN actually needs
            # (turn_plan.lane_offset_m, chosen to keep the car in-lane
            # whenever the corner allows it - see
            # _get_or_create_planned_turn). Gentle turns/curves plan with
            # the full lane offset and so never swerve at all; only
            # genuinely tight corners plan with a reduced (or negative,
            # opposing-lane) offset and swerve toward it.
            #
            # IMPORTANT: blended against distance_to_tangent_m (reaches
            # exactly 0 AT the tangent point), NOT remaining_to_junction_m
            # (which only reaches 0 at the TRUE junction - later than the
            # tangent point!). Using the wrong distance meant the blend
            # hadn't finished by the time we reached the tangent point,
            # leaving a residual gap that a forced snap-to-target_factor
            # then had to paper over - producing exactly the position jump
            # this blend exists to prevent. Now the blend completes
            # naturally, continuously, precisely AT the hand-off point.
            full_lane_offset_m = seg.width / 4
            target_factor = (turn_plan.lane_offset_m / full_lane_offset_m) if full_lane_offset_m > 0 else 0.0
            SWERVE_DISTANCE_M = 25.0
            blend = max(0.0, min(1.0, distance_to_tangent_m / SWERVE_DISTANCE_M))
            # blend=1.0 (far from tangent point) -> stay at full lane;
            # blend=0.0 (exactly at tangent point) -> reach target_factor
            planned_factor = target_factor + (1.0 - target_factor) * blend
            # If this plan's junction is the FIRST one after a hand-off
            # (recorded in _handle_segment_end / _advance_active_turn),
            # the car may not actually be at full lane offset (1.0) yet:
            # after a no-arc hand-off it is on the centerline (factor ~0,
            # the no-plan recovery blend still easing it back), and after
            # a tight-arc hand-off it is at whatever reduced factor that
            # arc used. The "stay at full lane" assumption above would
            # then snap the factor from its true hand-off value to the
            # planned one in a single frame - a sideways jump of up to
            # half a meter.
            #
            # So ease from the ACTUAL hand-off factor toward the planned
            # blend's value, completing the ease EXACTLY at the tangent
            # point: the arc that starts there is built from the tangent
            # point at the PLAN'S lane offset, so the car must be at
            # exactly target_factor when the arc begins - otherwise the
            # first arc frame teleports the car sideways to the arc's
            # start. The ease distance is the hand-off-to-tangent
            # distance capped at 20 m (a hand-off far from the tangent
            # point eases over a comfortable 20 m and then holds the
            # planned blend; a close hand-off eases over the shorter
            # distance it actually has). Both easings are anchored to
            # the same point (the tangent) and are smooth in it, so the
            # composed factor is continuous all the way to the arc.
            recovery_start_seg = getattr(self, '_recovery_start_seg_idx', None)
            if recovery_start_seg == self.seg_idx:
                recovery_start_progress = getattr(self, '_recovery_start_progress', 0.0)
                # Distance from the hand-off to the tangent point (fixed
                # for this plan) - the ease must span exactly this.
                tangent_progress = 1.0 - (turn_plan.from_tangent_offset_m / seg.length) if seg.length > 0 else 1.0
                handoff_to_tangent_m = max(0.1, seg.length * abs(tangent_progress - recovery_start_progress))
                ease_distance_m = min(20.0, handoff_to_tangent_m)
                distance_since_handoff_m = seg.length * abs(self.progress - recovery_start_progress)
                recovery_progress = max(0.0, min(1.0, distance_since_handoff_m / ease_distance_m))
            else:
                # No hand-off on this segment (or already moved on) - the
                # car is long since at full lane offset.
                recovery_progress = 1.0
            recovery_start = getattr(self, '_recovery_start_factor', 1.0)
            if recovery_progress < 1.0 and abs(recovery_start - 1.0) > 1e-6:
                self._lane_offset_factor = recovery_start + (planned_factor - recovery_start) * recovery_progress
            else:
                self._lane_offset_factor = planned_factor
            
            if distance_to_tangent_m <= distance_m:
                # This frame crosses the tangent point. Split the movement
                # instead of jumping: advance EXACTLY to the tangent point
                # on the current (straight) segment first, using the
                # normal plain-segment position/lane offset/heading code
                # (so it's pixel-consistent with every previous frame),
                # THEN start the arc and spend the leftover distance on it.
                # This gives a perfectly continuous hand-off with no jump.
                distance_to_tangent_m = max(0.0, distance_to_tangent_m)
                frac_to_tangent = distance_to_tangent_m / seg.length if seg.length > 0 else 0.0
                if self.forward:
                    self.progress += frac_to_tangent
                else:
                    self.progress -= frac_to_tangent
                # _lane_offset_factor was already computed just above via
                # the (now tangent-point-anchored) blend formula, using
                # this exact same distance_to_tangent_m - by the time we
                # finish moving that distance, we're at the tangent point
                # and the factor is already correct to a small fraction of
                # a percent (no forced override needed, which would
                # actually be slightly LESS precise than the natural
                # blend value).
                self._apply_plain_segment_position(seg)
                
                leftover_distance_m = distance_m - distance_to_tangent_m
                self.active_turn = turn_plan
                self.planned_turn = None
                self.planned_turn_key = None
                print(f"🔄 Starting turn: {self.seg_idx} → {turn_plan.to_seg_idx} "
                      f"(speed {self.speed * 3.6:.0f} km/h, target {turn_plan.target_speed_mps * 3.6:.0f} km/h, "
                      f"radius {turn_plan.radius:.1f}m, tangent_distance {turn_plan.tangent_distance_m:.1f}m)")
                if leftover_distance_m > 0:
                    self._advance_active_turn(leftover_distance_m, network)
                return
        else:
            # No turn planned here. This covers three distinct situations:
            #   (a) GENUINE dead end (no next segment at all) - the car
            #       brakes to a full stop before the node (below).
            #   (b) Unreachable signaled turn: a valid next segment exists
            #       (e.g. the left turn the blinker points at) but NO arc
            #       radius fits, even at the mechanical minimum - the
            #       blinker was set too late to make it. The car does NOT
            #       force-enter that segment: it slides past the junction
            #       on the straight continuation (or crashes into the
            #       obstacle at a dead-end T), blinker still on, looking
            #       for the next reachable decision point (see
            #       _handle_segment_end / docs/TURN_REWORK_PLAN.md).
            #   (c) Near-straight continuation (<10 deg, where
            #       plan_turn deliberately declines to build an arc) or a
            #       plain hand-off - a routine segment swap.
            #
            # Lane offset: first EASE back toward 1.0 (full right-lane)
            # from whatever factor the last completed turn actually used
            # (see _recovery_start_factor, set in _advance_active_turn on
            # completion) - not from an assumed 0. A turn that needed the
            # FULL lane offset (no swerve at all) correctly stays at 1.0
            # the whole time instead of being yanked down to a fresh,
            # wrong, distance-based value. THEN, for every junction the
            # car is about to cross WITHOUT an executed arc ((a) and (b)
            # and (c) alike), blend the factor down to 0 (centerline) over
            # the last ~20 m before the node: on the centerline both
            # segments share the exact node point, so the segment-end
            # hand-off has NO lateral jump, ever - regardless of how much
            # the two segments' lane-offset directions differ (this is
            # what used to teleport the car 2.4 m sideways when an
            # unreachable 90 deg turn was force-entered at the node).
            SWERVE_DISTANCE_M = 20.0
            # Measured from the hand-off point (segment + progress
            # recorded in _advance_active_turn when the last turn
            # completed), NOT from this segment's start - the hand-off
            # itself can already be well past the 20m mark (e.g. a tight
            # corner with a long tangent distance), which would otherwise
            # make the blend "already complete" the very next frame and
            # snap the lane factor straight to 1.0 instead of easing there.
            recovery_start_seg = getattr(self, '_recovery_start_seg_idx', None)
            if recovery_start_seg == self.seg_idx:
                recovery_start_progress = getattr(self, '_recovery_start_progress', 0.0)
                distance_since_handoff_m = seg.length * abs(self.progress - recovery_start_progress)
            else:
                # Already moved on to a later segment without needing
                # another turn - the blend is long done.
                distance_since_handoff_m = SWERVE_DISTANCE_M
            recovery_progress = max(0.0, min(1.0, distance_since_handoff_m / SWERVE_DISTANCE_M))
            recovery_start = getattr(self, '_recovery_start_factor', 1.0)
            recovery_factor = recovery_start + (1.0 - recovery_start) * recovery_progress
            # Approach blend to the centerline: 1.0 far from the junction,
            # 0.0 exactly at it. Taking the MIN with the recovery factor
            # keeps the recovery easing intact while it is still below the
            # approach value (junction close after a hand-off) and always
            # lands exactly on 0.0 at the node.
            approach_to_centerline = max(0.0, min(1.0, remaining_to_junction_m / SWERVE_DISTANCE_M))
            self._lane_offset_factor = min(recovery_factor, approach_to_centerline)
            
            # Only brake to a full stop here for a GENUINE dead end (no
            # next segment at all) - NOT merely "no smooth arc fit this
            # junction" (a valid next segment can still exist then; the
            # slide-past/crash logic in _handle_segment_end handles that
            # case). Conflating the two used to brake
            # the car to a stop and leave it permanently stuck just short
            # of perfectly ordinary junctions whenever no arc validated -
            # a real regression once introduced. A genuine dead end
            # brakes smoothly toward a full stop by the time we get there
            # instead of barreling into it at cruise speed (which used to
            # be an instant, jarring stop - or worse, on a long dead-end
            # segment, an actual position-teleport bug; see
            # _handle_segment_end).
            if getattr(self, '_pending_junction_is_dead_end', False) and self.speed > 0:
                braking_distance_m = (self.speed ** 2) / (2 * config.CAR_BRAKING)
                safety_margin_m = 5.0
                if remaining_to_junction_m <= braking_distance_m + safety_margin_m:
                    self.speed = max(0.0, self.speed - config.CAR_BRAKING * dt)
                    self.target_speed = 0.0
                    self._braking = True
        
        # Normal plain-segment movement (no turn starting this frame)
        distance_frac = distance_m / seg.length if seg.length > 0 else 0
        if self.forward:
            self.progress += distance_frac
        else:
            self.progress -= distance_frac
        
        # Check for segment end (only if NOT in an active turn)
        # If we have an active turn, the arc execution handles the transition
        if not self.active_turn:
            if self.progress >= 1.0:
                self._handle_segment_end(network, seg.end_node)
            elif self.progress <= 0.0:
                self._handle_segment_end(network, seg.start_node)
        
        seg = network.segments[self.seg_idx]
        self._apply_plain_segment_position(seg)
    
    def _apply_plain_segment_position(self, seg):
        """Compute x/y/heading from self.progress along a (straight) segment,
        including the right-hand lane offset. Shared by normal segment
        following and by the exact-tangent-point hand-off when a turn
        starts mid-frame (see _update_position_rails).
        """
        t = max(0.0, min(1.0, self.progress))
        self.x = seg.x1 + t * (seg.x2 - seg.x1)
        self.y = seg.y1 + t * (seg.y2 - seg.y1)
        
        # Lane offset (right side), blended down toward 0 (centerline) as
        # we approach a turn's tangent point - see _lane_offset_factor and
        # _check_and_plan_turn(). The tangent-fillet arc geometry is built
        # relative to the CENTERLINE, so the car must actually BE on the
        # centerline by the time it reaches the tangent point for a
        # jump-free hand-off; blending gets it there smoothly in advance
        # instead of driving centered the whole time.
        pppm = config.PIXELS_PER_METER
        lane_offset = (seg.width / 4) * pppm * self._lane_offset_factor if not seg.oneway else 0
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        seg_len = math.hypot(dx, dy)
        if seg_len > 0:
            nx = -dy / seg_len
            ny = dx / seg_len
            if self.forward:
                self.x -= nx * lane_offset
                self.y -= ny * lane_offset
            else:
                self.x += nx * lane_offset
                self.y += ny * lane_offset
        
        # Heading (smooth from segment direction, will be overridden during turn).
        # While a segment-end heading transition is active, the transition
        # owns the heading (it eases toward exactly this segment's
        # direction), so don't overwrite it here - that would snap the
        # heading back to the target in one frame and defeat the easing.
        if not self._heading_transition:
            if self.forward:
                self.heading = math.degrees(math.atan2(dx, dy))
            else:
                self.heading = math.degrees(math.atan2(-dx, -dy))
    
    def _get_or_create_planned_turn(self, network, junction_node: str):
        """Get the cached, ONE-TIME turn plan for the upcoming junction,
        (re)computing it only if the intended direction has changed since
        it was last planned (e.g. driver toggled a blinker).
        
        The plan's target speed and geometry are decided ONCE, at a fixed
        speed chosen purely from the turn's severity (TurningSystem.
        decide_target_speed_for_turn) - not from whatever speed the car
        happens to be going right now. This is what makes the tangent
        point and required braking distance stable, computable quantities
        instead of a moving target that shifts every frame while braking.
        
        Returns:
            TurnPlan if a valid fillet exists (at the chosen target speed,
            or a reduced fallback speed down to the mechanical minimum),
            else None (no valid turn, dead end, or roads too short for any
            realistic radius - segment-end instant-transition fallback
            handles that case).
        """
        # Get turn intention from driver
        if self.driver and hasattr(self.driver, 'pending_turn'):
            turn = self.driver.pending_turn or "straight"
        else:
            turn = "straight"
        
        next_seg_idx = network.choose_next_segment(self.seg_idx, junction_node, turn)
        
        if next_seg_idx is None or next_seg_idx == self.seg_idx:
            # A genuine dead end (no valid next segment at all) - as
            # opposed to "a next segment exists but no smooth arc could
            # be built for it" below, which is NOT a dead end and must
            # NOT brake to a stop the same way (see the caller in
            # _update_position_rails - conflating the two used to make
            # the car brake to a stop and get permanently stuck just
            # short of an ordinary junction whenever no arc validated,
            # even though a perfectly valid next segment existed and the
            # segment-end instant-transition fallback would have handled
            # it fine).
            self._pending_junction_is_dead_end = True
            self._pending_no_arc_next_seg = None
            self.planned_turn = None
            self.planned_turn_key = None
            return None
        
        self._pending_junction_is_dead_end = False
        # A plan (re)computed for this junction supersedes any stale
        # "no arc" state from an earlier evaluation of the same junction.
        self._pending_no_arc_next_seg = None
        
        key = (self.seg_idx, next_seg_idx, junction_node)
        if self.planned_turn_key == key and self.planned_turn is not None:
            return self.planned_turn
        
        # (Re)plan once: decide the ideal target speed from turn severity,
        # then fall back to progressively slower (tighter-radius) attempts
        # if the road is too short for the comfortable choice - down to
        # the absolute mechanical minimum before giving up entirely.
        turn_angle_deg = abs(network.get_exit_angle(self.seg_idx, next_seg_idx))
        cruise_speed_mps = config.CAR_SPEED
        ideal_speed = self.turning_system.decide_target_speed_for_turn(turn_angle_deg, cruise_speed_mps)
        min_speed = math.sqrt(self.turning_system.max_lateral_accel * self.turning_system.MIN_MECHANICAL_RADIUS_M)
        
        # Several speeds between ideal and the mechanical minimum - not
        # just 3 coarse steps - so "slow down further" has a real chance
        # to succeed within our OWN lane before ever considering the
        # opposing lane (see below). Descending, deduped, always ends
        # exactly at min_speed.
        candidate_speeds = [ideal_speed]
        steps = 6
        for i in range(1, steps + 1):
            frac = 1.0 - i / steps
            candidate_speeds.append(max(min_speed, ideal_speed * frac))
        seen = set()
        candidate_speeds = [round(s, 6) for s in candidate_speeds]
        candidate_speeds = [s for s in candidate_speeds if not (s in seen or seen.add(s))]
        
        # Other traffic may be coming the other way at any time - the
        # opposing lane is NEVER just a free way to take a turn faster.
        # It's only used as an absolute last resort, and only once EVERY
        # speed (all the way down to the mechanical minimum) has been
        # tried fully within our own lane first. This is why a real slow
        # crawl through even a tight hairpin stays entirely in-lane - it's
        # only speeds that would need to swerve that get rejected before
        # ever reaching the opposing-lane fallback.
        #
        # Full-street swerving (the reduced/centerline lane offsets below
        # full, while STILL >= 0 - i.e. still our own side) is kept for
        # when it's genuinely needed at a chosen speed and there's no
        # reason to think anything's coming - this is what gives
        # comfortable, natural-looking wide turns their own-lane swing on
        # open roads with no oncoming traffic yet modeled.
        from_seg = network.segments[self.seg_idx]
        to_seg = network.segments[next_seg_idx]
        # A one-way (single-lane) road has no lane to offset within -
        # _apply_plain_segment_position always forces lane_offset to 0
        # there, so the turn plan must too, or the fillet's own tangent
        # point ends up built for a lane position the car is never
        # actually at (a fixed offset-width mismatch every time, which on
        # short/tight geometry - e.g. a roundabout's ring - is large
        # enough to trip the teleportation watchdog at the hand-off).
        # Applies if EITHER end of the turn is one-way: a single fillet
        # geometry can't have a lane offset on one end and centerline on
        # the other, so if either side forces centerline, the whole arc
        # has to be built for centerline.
        full_lane_offset_m = (from_seg.width / 4) if not (from_seg.oneway or to_seg.oneway) else 0.0
        own_lane_fractions = [1.0, 0.5, 0.0]
        opposing_lane_fractions = [-0.5]
        
        plan = None
        for frac in own_lane_fractions:
            for candidate_speed in candidate_speeds:
                candidate = self.turning_system.plan_turn(
                    candidate_speed, self.seg_idx, next_seg_idx, junction_node, network,
                    lane_offset_m=full_lane_offset_m * frac,
                )
                if candidate:
                    plan = candidate
                    break
            if plan:
                break
        
        if plan is None:
            for frac in opposing_lane_fractions:
                for candidate_speed in candidate_speeds:
                    candidate = self.turning_system.plan_turn(
                        candidate_speed, self.seg_idx, next_seg_idx, junction_node, network,
                        lane_offset_m=full_lane_offset_m * frac,
                    )
                    if candidate:
                        plan = candidate
                        break
                if plan:
                    break
        
        self.planned_turn = plan
        self.planned_turn_key = key
        
        if plan is not None:
            # Record the car's ACTUAL state at plan-creation time as the
            # recovery start for the plan branch's lane-offset easing
            # (see _update_position_rails): the plan must ease from
            # wherever the car actually is right now, not from an
            # assumed full lane offset. (When the plan is created right
            # at a hand-off this matches the record _handle_segment_end
            # just wrote; when it is created mid-segment - e.g. the
            # driver set the blinker 15 m before the junction - it is
            # the only recovery record that exists.)
            self._recovery_start_factor = self._lane_offset_factor
            self._recovery_start_seg_idx = self.seg_idx
            self._recovery_start_progress = self.progress
        
        if plan is None:
            # The signaled segment exists but is physically unreachable
            # from here - remember it so _handle_segment_end can slide
            # past (or crash at a dead end) instead of force-entering it.
            self._pending_no_arc_next_seg = next_seg_idx
            if not getattr(self, '_warned_no_arc_for_node', None) == junction_node:
                self._warned_no_arc_for_node = junction_node
                print(f"⚠️ Cannot plan turn {self.seg_idx} → {next_seg_idx}: "
                      f"no radius fits within road length, even at the mechanical minimum "
                      f"and using the opposing lane - will slide past the junction "
                      f"(blinker stays on for the next reachable decision point)")
        
        return plan
    
    def _handle_segment_end(self, network, node_id: str):
        """Handle reaching end of current segment."""
        # Ask driver for turn decision
        if self.driver and hasattr(self.driver, 'pending_turn'):
            turn = self.driver.pending_turn or "straight"
        else:
            turn = "straight"
        
        next_seg = network.choose_next_segment(self.seg_idx, node_id, turn)
        
        # --- Slide past an UNREACHABLE turn (docs/TURN_REWORK_PLAN.md) ---
        # If the driver signaled a turn into a segment for which no arc
        # could be planned (the blinker was set too late - no radius fits
        # even at the mechanical minimum), the car must NOT force-enter
        # that segment: a real car that set its blinker 3 m before a 90°
        # corner simply cannot make that corner. It slides past the
        # junction on the straight continuation (smallest-angle exit),
        # blinker still on, to attempt the turn at the next reachable
        # decision point. Only if the road does not continue at all
        # (dead-end T) does the car physically crash into the obstacle -
        # a real, physical event (rapid deceleration to 0), never a
        # teleport or an instant stop from speed.
        if (self._pending_no_arc_next_seg is not None
                and next_seg == self._pending_no_arc_next_seg):
            continuation = network.choose_next_segment(self.seg_idx, node_id, "straight")
            if continuation is not None and continuation != self.seg_idx:
                print(f"🛞 Slide past unreachable turn {self.seg_idx} → "
                      f"{self._pending_no_arc_next_seg}: continuing on {continuation} "
                      f"(blinker stays on for the next reachable decision point)")
                next_seg = continuation
            else:
                print(f"💥 Dead-end T: no continuation for unreachable turn "
                      f"{self.seg_idx} → {self._pending_no_arc_next_seg} - "
                      f"crashing into the obstacle")
                self._crash_into_obstacle()
                return
        
        if next_seg is not None and next_seg != self.seg_idx:
            old_seg = self.seg_idx
            old_seg_len = network.segments[old_seg].length
            self.seg_idx = next_seg
            new_seg = network.segments[next_seg]
            self.forward = (new_seg.start_node == node_id)
            # Carry the OVERSHOOT across the node instead of discarding
            # it: this frame's movement already extended past the node
            # (progress > 1.0), so the car physically IS partway along
            # the new segment. Resetting progress to exactly 0.0/1.0
            # would lose up to a full frame of travel distance - which
            # shrinks the position step while the heading still changes
            # by the full segment angle, and on a short step that can
            # imply a turning radius below any real car's minimum
            # (the rotation watchdog is rightly sensitive to exactly
            # this). Scaling by the length ratio keeps the carried
            # distance in meters, not in progress fraction.
            overshoot_frac = max(0.0, self.progress - 1.0) if self.forward \
                else max(0.0, -self.progress)
            if old_seg_len > 0 and new_seg.length > 0:
                overshoot_m = overshoot_frac * old_seg_len
                carried_frac = min(0.25, overshoot_m / new_seg.length)
            else:
                carried_frac = 0.0
            self.progress = (carried_frac if self.forward else 1.0 - carried_frac)
            
            # Smooth the heading across the swap instead of snapping it
            # to the new segment's direction in one frame (matters for
            # sharp continuations; see _begin_heading_transition_for_segment).
            self._begin_heading_transition_for_segment(new_seg)
            
            # Record the hand-off for the lane-offset recovery blend: the
            # no-plan branch eases the factor back toward 1.0 (full
            # right-lane) over the next ~20 m starting from WHATEVER
            # FACTOR THE CAR HAD AT THE NODE - which for a no-arc
            # hand-off is ~0 (the approach blend lands the car on the
            # centerline exactly at the node, where both segments share
            # the exact same point). Without this record the recovery
            # blend would assume it starts from 1.0, and on a short
            # segment the "approach to the NEXT junction" blend would
            # immediately re-pull the factor down from 1.0 - a one-frame
            # factor jump (0 -> ~0.25) that moves the car sideways by
            # up to half a meter, tripping the teleportation watchdog.
            self._recovery_start_factor = self._lane_offset_factor
            self._recovery_start_seg_idx = next_seg
            self._recovery_start_progress = self.progress
            
            # Notify driver that we turned
            if self.driver and hasattr(self.driver, 'clear_blinker_if_turned'):
                self.driver.clear_blinker_if_turned(self, network, old_seg, next_seg)
        else:
            # Dead end - turn around. Nudge progress by a small FIXED
            # DISTANCE back onto the road, staying on the SAME side we
            # actually arrived at (node_id) - not a fixed 0.9/0.1
            # FRACTION of the segment based on the (already-flipped) new
            # forward direction, which put the car at the position ~1.0
            # or ~0.0 as if it had always been driving that way, rather
            # than the small nudge back from wherever it physically just
            # arrived. On a short segment the difference is tiny and
            # invisible; on a long one (e.g. a 150m roundabout spoke) the
            # old code silently teleported the car ~135-150m down the
            # road, tripping the teleportation watchdog and crashing the
            # game the first time anything actually drove all the way to
            # a long dead end.
            seg = network.segments[self.seg_idx]
            arrived_at_start = (node_id == seg.start_node)
            self.heading = (self.heading + 180) % 360
            self.forward = not self.forward
            nudge_m = min(2.0, seg.length / 4) if seg.length > 0 else 0.0
            nudge_frac = (nudge_m / seg.length) if seg.length > 0 else 0.0
            # Stay near the end we actually arrived at (node_id), just
            # nudged back onto the road - NOT wherever the new (flipped)
            # forward direction would imply for a normal in-progress drive.
            self.progress = nudge_frac if arrived_at_start else 1.0 - nudge_frac
            self.speed = 0
            if self.driver and hasattr(self.driver, 'blinker_left'):
                self.driver.blinker_left = False
                self.driver.blinker_right = False
                self.driver.pending_turn = None
    
    def _crash_into_obstacle(self):
        """Physical crash into the obstacle at a dead-end T-junction.
        
        A car that cannot make a signaled turn and whose road does not
        continue (dead-end T) does not stop magically: it hits the
        obstacle (wall / end of road) and decelerates rapidly to 0 over a
        short distance - a real physical event. No teleport, no instant
        stop from speed, no position jump (docs/TURN_REWORK_PLAN.md).
        
        The obstacle is treated as being one car length ahead of the
        node (the road's own end/shoulder), so the car keeps its
        segment-following position while its speed is drained by
        CRASH_DECEL over CRASH_DISTANCE - at any approach speed that is
        a full stop within a couple of meters, i.e. the car noses into
        the obstacle and comes to rest against it, exactly where the
        road ends.
        """
        CRASH_DECEL = 30.0      # m/s² - impact deceleration (harder than
                                # braking: the obstacle does the work)
        CRASH_DISTANCE_M = 3.0  # ~one car length of crumple distance
        # Frames of deceleration needed to reach 0 from the current speed.
        self._crash_frames_left = max(0, int(math.ceil(self.speed / CRASH_DECEL /
                                                      (1.0 / 60.0))))
        self._crash_decel = CRASH_DECEL
        # Cap: even at absurd speeds the crash takes at most this long.
        self._crash_frames_left = min(self._crash_frames_left, 60)
        self.target_speed = 0.0
        self._braking = True

    def _begin_heading_transition_for_segment(self, new_seg):
        """Start a smooth heading transition toward the new segment's
        direction after a segment-end hand-off that had no executed arc
        (i.e. the heading would otherwise snap in a single frame).
        
        The transition is a RATE-LIMITED rotation, not a fixed-time
        interpolation: every frame the nose rotates toward the target at
        min(planned rate, v / _heading_transition_radius_m). The
        speed-dependent cap is what keeps the motion physical no matter
        how the speed changes mid-transition - rotating at omega while
        moving at v sweeps a circle of radius v/omega, and the cap keeps
        that at or above 4.0 m (comfortably above the PhysicsValidator's
        3.0 m minimum). A car that brakes while the transition is still
        running therefore rotates more slowly (a real car does exactly
        this), and a stationary car does not rotate at all. Small
        differences (gentle continuations) are applied directly.
        """
        dx = new_seg.x2 - new_seg.x1
        dy = new_seg.y2 - new_seg.y1
        if self.forward:
            target_heading = math.degrees(math.atan2(dx, dy))
        else:
            target_heading = math.degrees(math.atan2(-dx, -dy))
        diff = (target_heading - self.heading + 180) % 360 - 180
        if abs(diff) < 0.5:
            # Negligible - apply directly, no transition needed.
            return
        # Planned duration: at least 0.3 s, at most 120 deg/s average.
        duration = max(0.3, abs(diff) / 120.0)
        self._heading_transition = True
        self._heading_transition_elapsed = 0.0
        self._heading_transition_duration = duration
        self._heading_transition_remaining = diff
        self._heading_target = target_heading

    def _update_heading_transition(self, dt: float):
        """Advance the rate-limited heading transition (see
        _begin_heading_transition_for_segment)."""
        if not self._heading_transition:
            return
        # Physical cap on the rotation rate: keep the implied radius
        # (v / omega) at or above _heading_transition_radius_m at ALL
        # times, including while the car is braking through the
        # transition.
        max_rate = (self.speed / self._heading_transition_radius_m) * (180.0 / math.pi)
        # Planned rate: finish the remaining rotation within the
        # remaining scheduled time (grows if the cap delayed us).
        time_left = max(1e-6, self._heading_transition_duration - self._heading_transition_elapsed)
        planned_rate = abs(self._heading_transition_remaining) / time_left
        rate = min(planned_rate, max_rate)
        step = rate * dt
        if step >= abs(self._heading_transition_remaining) - 1e-9:
            self.heading = self._heading_target % 360
            self._heading_transition_remaining = 0.0
            self._heading_transition = False
        else:
            self.heading = (self.heading + math.copysign(step, self._heading_transition_remaining)) % 360
            self._heading_transition_remaining -= math.copysign(step, self._heading_transition_remaining)
            self._heading_transition_elapsed += dt
    
    # --- Utility ---
    
    def snap_to_road(self, network):
        """Snap car to nearest road segment."""
        pppm = config.PIXELS_PER_METER
        best_seg = 0
        best_dist = float("inf")
        best_t = 0.5
        
        for idx, seg in enumerate(network.segments):
            dx = seg.x2 - seg.x1
            dy = seg.y2 - seg.y1
            length_sq = dx * dx + dy * dy
            if length_sq == 0:
                continue
            t = max(0, min(1, ((self.x - seg.x1) * dx + (self.y - seg.y1) * dy) / length_sq))
            proj_x = seg.x1 + t * dx
            proj_y = seg.y1 + t * dy
            dist = math.hypot(self.x - proj_x, self.y - proj_y)
            if dist < best_dist:
                best_dist = dist
                best_seg = idx
                best_t = t
        
        self.seg_idx = best_seg
        self.progress = best_t
        seg = network.segments[best_seg]
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        seg_heading = math.degrees(math.atan2(dx, dy))
        diff = abs((self.heading - seg_heading + 180) % 360 - 180)
        self.forward = diff < 90
    
    def teleport_random(self, network):
        """Teleport to random road location.
        
        Note: Caller should notify PhysicsValidator.skip_next_frames() if using validation.
        """
        rx, ry, rh, seg_idx, node_id = network.random_road_point()
        self.x = rx
        self.y = ry
        self.heading = rh
        self.seg_idx = seg_idx
        self.progress = 0.5
        self.speed = 0
        self.target_speed = 0
        
        # Determine forward direction
        seg = network.segments[seg_idx]
        dx = seg.x2 - seg.x1
        dy = seg.y2 - seg.y1
        seg_heading = math.degrees(math.atan2(dx, dy))
        diff = abs((self.heading - seg_heading + 180) % 360 - 180)
        self.forward = diff < 90
        
        self.active_turn = None
        self.planned_turn = None
        self.planned_turn_key = None
        self._pending_no_arc_next_seg = None
        self._lane_offset_factor = 1.0
        self._recovery_start_factor = 1.0
        self._recovery_start_seg_idx = None
        self._recovery_start_progress = 0.0
        self._crash_frames_left = 0
        self._crash_decel = 0.0
        self._heading_transition = False
        # Apply the normal right-lane offset immediately - see the
        # matching comment in teleport_to_named_point().
        self._apply_plain_segment_position(seg)
        if self.bicycle_nav is not None:
            self.bicycle_nav.reset()
        
        # Clear trail
        self.trail.clear()

    def teleport_to_named_point(self, network, name: str):
        """Teleport to a deterministic named start point (synthetic test
        maps only — see RoadNetwork.start_points / test_maps.py).

        Note: Caller should notify PhysicsValidator.reset_car_state() if
        using validation.
        """
        x, y, heading, seg_idx, forward = network.get_start_point(name)
        self.heading = heading
        self.seg_idx = seg_idx
        self.progress = 0.0 if forward else 1.0
        self.forward = forward
        self.speed = 0
        self.target_speed = 0
        self.active_turn = None
        self.planned_turn = None
        self.planned_turn_key = None
        self._pending_no_arc_next_seg = None
        self._lane_offset_factor = 1.0
        self._recovery_start_factor = 1.0
        self._recovery_start_seg_idx = None
        self._recovery_start_progress = 0.0
        self._crash_frames_left = 0
        self._crash_decel = 0.0
        self._heading_transition = False
        # Apply the normal right-lane offset immediately (matching what
        # continuous driving would show) instead of leaving the car at
        # the raw centerline node coordinates - otherwise the very next
        # physics frame "snaps" it sideways into its lane, which the
        # (correctly strict) teleportation watchdog flags as a real jump.
        self._apply_plain_segment_position(network.segments[seg_idx])
        if self.bicycle_nav is not None:
            self.bicycle_nav.reset()
        self.trail.clear()

    def is_on_road(self, network) -> bool:
        """Check if the car (all four corners, not just its center) is on
        any road.

        Delegates to RoadNetwork.is_car_on_road(), which tests the car's
        four bounding-box corners against the exact same paved-area polygon
        that gets rendered (rounded bends and junction fillets included).
        A center-point-only check (the old behaviour, "bicycle-style") let
        the car's body/wheels overhang the road edge without being detected
        - e.g. at a roundabout the center could stay on the road while the
        outer wheels rode the curb. Checking the corners catches that.
        """
        return network.is_car_on_road(self.x, self.y, self.heading)
    
    # --- Rendering ---
    
    def draw(self, surface: pygame.Surface, camera):
        """Draw the car sprite."""
        sx, sy = camera.world_to_screen(self.x, self.y)
        sx, sy = int(sx), int(sy)
        scale = camera.zoom
        
        # Calculate desired size in screen pixels
        # CAR_LENGTH and CAR_WIDTH are in meters
        car_length_px = config.CAR_LENGTH * config.PIXELS_PER_METER * scale
        car_width_px = config.CAR_WIDTH * config.PIXELS_PER_METER * scale
        
        # Load base sprite (high-res version)
        base_sprite = self._get_base_sprite()
        
        # Scale sprite to match world dimensions
        scaled_sprite = pygame.transform.scale(base_sprite, (int(car_width_px), int(car_length_px)))
        
        # Rotate sprite
        rotated = pygame.transform.rotate(scaled_sprite, -self.heading)
        
        # Center and draw
        rect = rotated.get_rect(center=(sx, sy))
        surface.blit(rotated, rect)
        
        # Draw dynamic lights on top (blinkers)
        if self.driver and self.driver.get_name() in ("RAILS", "BICYCLE"):
            self._draw_blinkers(surface, sx, sy, scale)
    
    def _get_base_sprite(self) -> pygame.Surface:
        """Load and cache the base car sprite (high-res version)."""
        import os
        from PIL import Image
        
        sprite_file = 'car_64x128.png'  # Use high-res version as base
        
        # Check cache
        if sprite_file not in Car._sprite_cache:
            # Load sprite using PIL (pygame doesn't have PNG support)
            sprite_path = os.path.join(os.path.dirname(__file__), '..', 'assets', sprite_file)
            if os.path.exists(sprite_path):
                # Load with PIL
                pil_image = Image.open(sprite_path).convert('RGBA')
                # Convert PIL image to pygame surface
                mode = pil_image.mode
                size = pil_image.size
                data = pil_image.tobytes()
                surf = pygame.image.fromstring(data, size, mode)
                Car._sprite_cache[sprite_file] = surf
            else:
                # Fallback: create simple colored rectangle if sprite not found
                size = (64, 128)
                surf = pygame.Surface(size, pygame.SRCALPHA)
                pygame.draw.rect(surf, (180, 30, 30), surf.get_rect())
                Car._sprite_cache[sprite_file] = surf
        
        return Car._sprite_cache[sprite_file]
    
    def _draw_headlights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        for sign in (-1, 1):
            hx = sx + math.sin(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            hx += math.cos(rad) * sign * half_wid * 0.6
            hy = sy - math.cos(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            hy += math.sin(rad) * sign * half_wid * 0.6
            pygame.draw.circle(surface, (255, 255, 220), (int(hx), int(hy)), 2)
    
    def _draw_taillights(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        for sign in (-1, 1):
            tx = sx - math.sin(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            tx += math.cos(rad) * sign * half_wid * 0.6
            ty = sy + math.cos(rad) * config.CAR_LENGTH / 2 * config.PIXELS_PER_METER * scale
            ty += math.sin(rad) * sign * half_wid * 0.6
            color = (255, 40, 0) if self._braking else (150, 0, 0)
            pygame.draw.circle(surface, color, (int(tx), int(ty)), 2)
    
    def _draw_blinkers(self, surface: pygame.Surface, sx: float, sy: float, scale: float):
        """Draw blinker lights (orange, flashing) in RAILS mode."""
        if not self.driver or not hasattr(self.driver, 'blinker_left'):
            return
        
        if not (self.driver.blinker_left or self.driver.blinker_right):
            return
        
        # Flash with 0.5s period
        import time
        if (time.time() % 0.5) > 0.25:
            return
        
        half_wid = (config.CAR_WIDTH / 2) * config.PIXELS_PER_METER * scale
        rad = math.radians(self.heading)
        
        for side in ((-1, self.driver.blinker_left), (1, self.driver.blinker_right)):
            sign, active = side
            if not active:
                continue
            bx = sx + math.sin(rad) * config.CAR_LENGTH / 3 * config.PIXELS_PER_METER * scale
            bx += math.cos(rad) * sign * half_wid * 0.8
            by = sy - math.cos(rad) * config.CAR_LENGTH / 3 * config.PIXELS_PER_METER * scale
            by += math.sin(rad) * sign * half_wid * 0.8
            pygame.draw.circle(surface, (255, 180, 0), (int(bx), int(by)), 3)
