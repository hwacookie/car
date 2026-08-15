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
        
        # Visual state
        self._braking = False
        self._accelerating = False
        
        # Smooth heading transition
        self._heading_transition = False
        self._heading_transition_progress = 0.0
        self._heading_start = 0.0
        self._heading_target = 0.0
        self._heading_transition_duration = 0.3
        
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
        if self.driver and self.driver.get_name() == "RAILS":
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
            
            # Remember the lane-offset factor the just-completed turn
            # actually used (self._lane_offset_factor is untouched by arc
            # execution, so it's still exactly the plan's target_factor
            # here) - the recovery blend on the new segment eases FROM
            # this value toward 1.0, instead of assuming it always starts
            # from 0. Without this, a turn that used the FULL lane offset
            # (no swerve needed at all) would still get yanked down to a
            # fresh, wrong, distance-based factor on the very next frame.
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
            self._lane_offset_factor = target_factor + (1.0 - target_factor) * blend
            
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
            # No turn planned here (straight continuation, dead end, or no
            # radius fits even at the mechanical minimum). Blend the lane
            # offset back toward 1.0 (full right-lane) as we get further
            # into the segment, EASING FROM WHATEVER FACTOR THE LAST
            # COMPLETED TURN ACTUALLY USED (see _recovery_start_factor,
            # set in _advance_active_turn on completion) - not from an
            # assumed 0. A turn that needed the FULL lane offset (no
            # swerve at all) correctly stays at 1.0 the whole time instead
            # of being yanked down to a fresh, wrong, distance-based value.
            # (The old formula also blended down near the far end of the
            # segment "just in case" another turn was coming - redundant
            # now, since the "if turn_plan" branch above already handles
            # that with a precise, tangent-point-anchored blend.)
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
            self._lane_offset_factor = recovery_start + (1.0 - recovery_start) * recovery_progress
            
            # No turn plan means either a genuine dead end or nothing
            # geometrically fits (even at the mechanical minimum) - in
            # BOTH cases there's no arc to smoothly carry us past this
            # junction, so brake toward a full stop by the time we get
            # there instead of barreling into it at cruise speed (which
            # used to be an instant, jarring stop - or worse, on a long
            # dead-end segment, an actual position-teleport bug; see
            # _handle_segment_end). Same full-strength braking math as
            # the turn-approach case above, just targeting speed 0 at
            # the junction itself rather than a fixed turn speed at the
            # tangent point.
            if self.speed > 0:
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
        
        # Heading (smooth from segment direction, will be overridden during turn)
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
            self.planned_turn = None
            self.planned_turn_key = None
            return None
        
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
        
        if plan is None:
            if not getattr(self, '_warned_no_arc_for_node', None) == junction_node:
                self._warned_no_arc_for_node = junction_node
                print(f"⚠️ Cannot plan turn {self.seg_idx} → {next_seg_idx}: "
                      f"no radius fits within road length, even at the mechanical minimum "
                      f"and using the opposing lane")
        
        return plan
    
    def _handle_segment_end(self, network, node_id: str):
        """Handle reaching end of current segment."""
        # Ask driver for turn decision
        if self.driver and hasattr(self.driver, 'pending_turn'):
            turn = self.driver.pending_turn or "straight"
        else:
            turn = "straight"
        
        next_seg = network.choose_next_segment(self.seg_idx, node_id, turn)
        
        if next_seg is not None and next_seg != self.seg_idx:
            old_seg = self.seg_idx
            self.seg_idx = next_seg
            new_seg = network.segments[next_seg]
            self.forward = (new_seg.start_node == node_id)
            self.progress = 0.0 if self.forward else 1.0
            
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
    
    def _update_heading_transition(self, dt: float):
        """Smooth heading interpolation."""
        self._heading_transition_progress += dt / self._heading_transition_duration
        
        if self._heading_transition_progress >= 1.0:
            self._heading_transition = False
            self.heading = self._heading_target
        else:
            t = self._heading_transition_progress
            t = t * t * (3 - 2 * t)  # smoothstep
            
            diff = self._heading_target - self._heading_start
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            self.heading = (self._heading_start + t * diff) % 360
    
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
        self._lane_offset_factor = 1.0
        self._recovery_start_factor = 1.0
        self._recovery_start_seg_idx = None
        self._recovery_start_progress = 0.0
        # Apply the normal right-lane offset immediately - see the
        # matching comment in teleport_to_named_point().
        self._apply_plain_segment_position(seg)
        
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
        self._lane_offset_factor = 1.0
        self._recovery_start_factor = 1.0
        self._recovery_start_seg_idx = None
        self._recovery_start_progress = 0.0
        # Apply the normal right-lane offset immediately (matching what
        # continuous driving would show) instead of leaving the car at
        # the raw centerline node coordinates - otherwise the very next
        # physics frame "snaps" it sideways into its lane, which the
        # (correctly strict) teleportation watchdog flags as a real jump.
        self._apply_plain_segment_position(network.segments[seg_idx])
        self.trail.clear()

    def is_on_road(self, network) -> bool:
        """Check if car is currently on any road.

        Delegates entirely to RoadNetwork.is_on_road(), which tests
        against the exact same paved-area polygon that gets rendered
        (rounded bends and junction fillets included). This used to be a
        separate, independently-approximated rectangle+circle check here
        that could disagree with what was actually drawn (e.g. flagging
        a car as off-road while it was visibly still on the smooth
        rendered curve through a bend) - now there's a single source of
        truth for "where the road actually is".
        """
        return network.is_on_road(self.x, self.y)
    
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
        if self.driver and self.driver.get_name() == "RAILS":
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
