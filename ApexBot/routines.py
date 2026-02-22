from math import atan2
import math
import numpy as np

from utils import *


# This file holds all of the mechanical tasks, called "routines", that the bot can do

gravity = np.array([0, 0, -650])
# Aerial constants
max_speed = 2300
boost_accel = 1060
throttle_accel = 200 / 3
boost_per_second = 30

# Jump constants
jump_speed = 291.667
jump_acc = 1458.3333
jump_min_duration = 0.025
jump_max_duration = 0.2

class atba():
    # An example routine that just drives towards the ball at max speed
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)


class aerial_shot():
    # Very similar to jump_shot(), but instead designed to hit targets above 300uu
    # ***This routine is a WIP*** It does not currently hit the ball very hard, nor does it like to be accurate above 600uu or so
    def __init__(self, ball_location, intercept_time, shot_vector, ratio):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we hit the ball at
        self.intercept = self.ball_location - (self.shot_vector * 110)
        # dictates when (how late) we jump, much later than in jump_shot because we can take advantage of a double jump
        self.jump_threshold = 600
        # what time we began our jump at
        self.jump_time = 0
        # If we need a second jump we have to let go of the jump button for 3 frames, this counts how many frames we have let go for
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.01, 10.0)

        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(np.dot(np.cross(self.shot_vector, np.array([0, 0, 1])), car_to_ball))

        car_to_intercept = self.intercept - agent.me.location
        # car_to_intercept_perp = car_to_intercept.cross((0, 0, side_of_shot))  # perpendicular
        car_to_intercept_perp = np.cross(car_to_intercept, np.array([0, 0, side_of_shot]))

        distance_remaining = magnitude(flatten(car_to_intercept))

        speed_required = distance_remaining / time_remaining
        # When still on the ground we pretend gravity doesn't exist, for better or worse
        acceleration_required = backsolve(self.intercept, agent.me, time_remaining, 0 if self.jump_time == 0 else 325)
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        # adjustment = car_to_intercept.angle(self.shot_vector) * distance_remaining / 1.57  # size of adjustment
        adjustment = angle_between(car_to_intercept, self.shot_vector) * distance_remaining / 1.57

        adjustment *= (cap(self.jump_threshold - (acceleration_required[2]), 0.0,
                           self.jump_threshold) / self.jump_threshold)  # factoring in how close to jump we are
        # we don't adjust the final target if we are already jumping
        # final_target = self.intercept + ((car_to_intercept_perp.normalize() * adjustment) if self.jump_time == 0 else 0)
        norm_perp, _ = normalize(car_to_intercept_perp)
        final_target = self.intercept + ((norm_perp * adjustment) if self.jump_time == 0 else 0)

        # Some extra adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        #if abs(agent.me.location[1]) > 5120: final_target[0] = cap(final_target[0], -750, 750)
        if in_goal_area(agent):
            final_target[0] = cap(final_target[0], -750, 750)

            final_target[1] = cap(final_target[1], -5050, 5050)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.intercept)
        agent.line(self.intercept - np.array([0, 0, 100]), self.intercept + np.array([0, 0, 100]), [255, 0, 0])
        agent.line(final_target - np.array([0, 0, 100]), final_target + np.array([0, 0, 100]), [0, 255, 0])

        angles = defaultPD(agent, local_final_target)

        if self.jump_time == 0:
            defaultThrottle(agent, speed_required)
            agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
            agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake
            if acceleration_required[2] > self.jump_threshold:
                # Switch into the jump when the upward acceleration required reaches our threshold, hopefully we have aligned already...
                self.jump_time = agent.time
        else:
            time_since_jump = agent.time - self.jump_time

            # While airborne we boost if we're within 30 degrees of our local acceleration requirement
            # if agent.me.airborne and local_acceleration_required.magnitude() * time_remaining > 100:
            if agent.me.airborne and magnitude(local_acceleration_required) * time_remaining > 100:
                angles = defaultPD(agent, local_acceleration_required)
                if abs(angles[0]) + abs(angles[1]) < 0.5:
                    agent.controller.boost = True
            if self.counter == 0 and (time_since_jump <= 0.2 and local_acceleration_required[2] > 0):
                # hold the jump button up to 0.2 seconds to get the most acceleration from the first jump
                agent.controller.jump = True
            elif time_since_jump > 0.2 and self.counter < 3:
                # Release the jump button for 3 ticks
                agent.controller.jump = False
                self.counter += 1
            elif local_acceleration_required[2] > 300 and self.counter == 3:
                # the acceleration from the second jump is instant, so we only do it for 1 frame
                agent.controller.jump = True
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                self.counter += 1

        if raw_time_remaining < -0.25 or not shot_valid(agent, self):
            agent.pop()
            agent.push(recovery())


class flip():
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False):
        self.vector, _ = normalize(vector)
        self.pitch = abs(self.vector[0]) * -sign(self.vector[0])
        self.yaw = abs(self.vector[1]) * sign(self.vector[1])
        self.cancel = cancel
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        if elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed >= 0.15 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.pop()
            agent.push(recovery())


class goto():
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    # TODO - slow down if target is inside our turn radius
    def __init__(self, target, vector=None, direction=1, urgent=False):
        self.target = target
        self.vector = vector
        self.direction = direction
        self.urgent = urgent

    def run(self, agent):
        car_to_target = self.target - agent.me.location
        # distance_remaining = car_to_target.flatten().magnitude()
        distance_remaining = magnitude(flatten(car_to_target))

        agent.line(self.target - np.array([0, 0, 500]), self.target + np.array([0, 0, 500]), [255, 0, 255])

        if self.vector is not None:
            # See commends for adjustment in jump_shot or aerial for explanation
            side_of_vector = sign(np.dot(np.cross(self.vector, np.array([0, 0, 1])), car_to_target))

            # car_to_target_perp = car_to_target.cross((0, 0, side_of_vector)).normalize()
            car_to_target_perp, _ = normalize(np.cross(car_to_target, np.array([0, 0, side_of_vector])))

            # adjustment = car_to_target.angle(self.vector) * distance_remaining / 3.14
            adjustment = angle_between(car_to_target, self.vector) * distance_remaining / 3.14

            final_target = self.target + (car_to_target_perp * adjustment)
        else:
            final_target = self.target

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        #if abs(agent.me.location[1]) > 5120: final_target[0] = cap(final_target[0], -750, 750)
        if in_goal_area(agent):
            final_target[0] = cap(final_target[0], -750, 750)
            final_target[1] = cap(final_target[1], -5050, 5050)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target, self.direction)
        defaultThrottle(agent, 2300, self.direction)

        agent.controller.boost = True if self.urgent and distance_remaining > 1500 else False
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        velocity = 1 + magnitude(agent.me.velocity)

        """
        demo_coming, democar = detect_demo(agent)
        if demo_coming:
            agent.push(avoid_demo(democar))
        """

        if distance_remaining < 350:
            agent.pop()
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and distance_remaining / velocity > 2.0:
            if agent.me.up[2] < 0.9 or agent.me.airborne:
                agent.push(flip(local_target))
            elif abs(agent.controller.yaw) < 0.2:
                if agent.me.boost > 20 and self.urgent:
                    agent.push(boost_wave_dash())
                else:
                    agent.push(wave_dash())
        elif abs(angles[1]) > 2.8 and velocity < 200:
            agent.push(flip(local_target, True))
        elif agent.me.airborne:
            agent.push(recovery(self.target))


class goto_boost():
    # very similar to goto() but designed for grabbing boost
    # if a target is provided the bot will try to be facing the target as it passes over the boost
    # UPDATED: Now uses aggressive speedflips
    def __init__(self, boost, target=None):
        self.boost = boost
        self.target = target
        self.start = -1

    def run(self, agent):
        if self.start == -1:
            self.start = agent.time
        elapsed = agent.time - self.start
        car_to_boost = self.boost.location - agent.me.location
        distance_remaining = magnitude(flatten(car_to_boost))

        agent.line(self.boost.location - np.array([0, 0, 500]), self.boost.location + np.array([0, 0, 500]), [0, 255, 0])
        if self.target is not None:
            # vector = (self.target - self.boost.location).normalize()
            vector, _ = normalize(self.target - self.boost.location)

            # side_of_vector = sign(vector.cross((0, 0, 1)).dot(car_to_boost))
            side_of_vector = sign(np.dot(np.cross(vector, np.array([0, 0, 1])), car_to_boost))

            # car_to_boost_perp = car_to_boost.cross((0, 0, side_of_vector)).normalize()
            car_to_boost_perp, _ = normalize(np.cross(car_to_boost, np.array([0, 0, side_of_vector])))

            # adjustment = car_to_boost.angle(vector) * distance_remaining / 3.14
            adjustment = angle_between(car_to_boost, vector) * distance_remaining / 3.14

            final_target = self.boost.location + (car_to_boost_perp * adjustment)
            car_to_target = magnitude(self.target - agent.me.location)
        else:
            adjustment = 9999
            car_to_target = 0
            final_target = np.array([self.boost.location[0], self.boost.location[1], self.boost.location[2]])

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        #if abs(agent.me.location[1]) > 5120: final_target[0] = cap(final_target[0], -750, 750)
        if in_goal_area(agent):
            final_target[0] = cap(final_target[0], -750, 750)
            final_target[1] = cap(final_target[1], -5050, 5050)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        agent.controller.boost = self.boost.large if abs(angles[1]) < 0.3 else False
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        velocity = 1 + magnitude(agent.me.velocity)

        """
        demo_coming, democar = detect_demo(agent)
        if demo_coming:
            agent.push(avoid_demo(democar))
        """

        if elapsed > 3:
            agent.pop()

        if self.boost.active == False or agent.me.boost >= 99.0 or distance_remaining < 350:
            agent.pop()
        elif agent.me.airborne:
            agent.push(recovery(self.target))
        elif abs(angles[1]) < 0.1 and velocity > 600:
             # Aggressive flip usage
             if distance_remaining > 1800 and not agent.me.airborne:
                 agent.push(speed_flip(self.boost.location)) # Speedflip for long distance
             elif distance_remaining > 600 and not agent.me.airborne:
                 agent.push(flip(local_target)) # Normal flip for mid distance
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and (
                distance_remaining / velocity > 2.0 or (adjustment < 90 and car_to_target / velocity > 2.0)):
            # to prevent oversteering
            if abs(agent.controller.yaw) < 0.2:
                if agent.me.up[2] < 0.9 or agent.me.airborne:
                    agent.push(flip(local_target))
                elif agent.me.boost > 20:
                    agent.push(boost_wave_dash())
                else:
                    agent.push(wave_dash())
                #agent.push(flip(local_target))


class jump_shot():
    # Hits a target point at a target time towards a target direction
    # Target must be no higher than 300uu unless you're feeling lucky
    # TODO - speed
    def __init__(self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we dodge at
        # 173 is the 93uu ball radius + a bit more to account for the car's hitbox
        self.dodge_point = self.ball_location - (self.shot_vector * 173)
        # Ratio is how aligned the car is. Low ratios (<0.5) aren't likely to be hit properly
        self.ratio = ratio
        # whether the car should attempt this backwards
        self.direction = direction
        # Intercept speed not implemented
        self.speed_desired = speed
        # controls how soon car will jump based on acceleration required. max 584
        # bigger = later, which allows more time to align with shot vector
        # smaller = sooner
        self.jump_threshold = 400
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(np.dot(np.cross(self.shot_vector, np.array([0, 0, 1])), car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = np.cross(car_to_dodge_point, np.array([0, 0, side_of_shot]))

        distance_remaining = magnitude(car_to_dodge_point)

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650)
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        # adjustment = car_to_dodge_point.angle(self.shot_vector) * distance_remaining / 2.0  # size of adjustment
        adjustment = angle_between(car_to_dodge_point, self.shot_vector) * distance_remaining / 2.0

        adjustment *= (cap(self.jump_threshold - (acceleration_required[2]), 0.0,
                           self.jump_threshold) / self.jump_threshold)  # factoring in how close to jump we are
        # we don't adjust the final target if we are already jumping

        norm_perp, _ = normalize(car_to_dodge_perp)
        final_target = self.dodge_point + (
            (norm_perp * adjustment) if not self.jumping else 0) + np.array([0, 0, 50])

        # Ensuring our target isn't too close to the sides of the field, where our car would get messed up by the radius of the curves

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        #if abs(agent.me.location[1]) > 5120: final_target[0] = cap(final_target[0], -750, 750)
        if in_goal_area(agent):
            final_target[0] = cap(final_target[0], -750, 750)
            final_target[1] = cap(final_target[1], -5050, 5050)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.dodge_point)
        agent.line(self.dodge_point - np.array([0, 0, 100]), self.dodge_point + np.array([0, 0, 100]), [255, 0, 0])
        agent.line(final_target - np.array([0, 0, 100]), final_target + np.array([0, 0, 100]), [0, 255, 0])

        # Calling our drive utils to get us going towards the final target
        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.line(agent.me.location, agent.me.location + (self.shot_vector * 200), [255, 255, 255])

        agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 and self.direction == 1 else agent.controller.handbrake

        if not self.jumping:
            if raw_time_remaining <= 0.0 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent,
                                                                                                            self):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif local_acceleration_required[2] > self.jump_threshold and local_acceleration_required[
                2] > magnitude(flatten(local_acceleration_required)):
                # Switch into the jump when the upward acceleration required reaches our threshold, and our lateral acceleration is negligible
                self.jumping = True
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 150)) or raw_time_remaining <= -0.9 or (
                    not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and local_acceleration_required[2] > 0.0 and raw_time_remaining > 0.083:
                # Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                # make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                # dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -sign(vector[0])
                    self.y = abs(vector[1]) * sign(vector[1]) * self.direction
                    self.dodging = True
                # simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0


class speed_flip():
    def __init__(self, target):
        self.target = target
        self.start_time = -1
        self.jump_timer = 0
        self.phase = 0 # 0=Drive, 1=Jump, 2=Dodge, 3=Cancel

    def run(self, agent):
        if self.start_time == -1:
            self.start_time = agent.time

        elapsed = agent.time - self.start_time

        # Calculate direction
        local_target = agent.me.local(self.target - agent.me.location)
        defaultPD(agent, local_target)

        # Phase 0: Drive until speed or time condition
        if self.phase == 0:
            defaultThrottle(agent, 2300)
            if magnitude(agent.me.velocity) > 1050: # Trigger speed flip
                self.phase = 1
                self.jump_timer = agent.time
            # Timeout safeguard
            if elapsed > 2.0:
                 agent.pop()

        # Phase 1: First Jump
        elif self.phase == 1:
            defaultThrottle(agent, 2300)
            jump_elapsed = agent.time - self.jump_timer
            if jump_elapsed < 0.05:
                agent.controller.jump = True
                agent.controller.pitch = 0 # Neutral jump
            elif jump_elapsed < 0.1:
                agent.controller.jump = False
                agent.controller.pitch = 0
            else:
                self.phase = 2
                self.jump_timer = agent.time # Reset for dodge

        # Phase 2: Dodge (Diagonal)
        elif self.phase == 2:
            defaultThrottle(agent, 2300)
            # Diagonal front flip
            # Pitch -1 = Forward
            agent.controller.jump = True
            agent.controller.pitch = -1
            # Roll/Yaw towards target
            angle = atan2(local_target[1], local_target[0])
            direction = sign(angle) if abs(angle) > 0.1 else 1
            agent.controller.roll = 0
            agent.controller.yaw = direction
            self.phase = 3
            self.jump_timer = agent.time

        # Phase 3: Cancel
        elif self.phase == 3:
            defaultThrottle(agent, 2300)
            cancel_elapsed = agent.time - self.jump_timer

            # Hold cancel (Pitch 1 = Back)
            if cancel_elapsed < 0.6: # Cancel duration
                agent.controller.pitch = 1
                agent.controller.jump = False
                # Continue rolling/yawing? Usually just hold pitch back
                agent.controller.roll = 0
                agent.controller.yaw = 0
                agent.controller.handbrake = False
                # Optionally air roll to land wheels down
            else:
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())


class kickoff():
    # A simple 1v1 kickoff that just drives up behind the ball and dodges
    # misses the boost on the slight-offcenter kickoffs haha
    def run(self, agent):
        target = agent.ball.location + np.array([0, 200 * side(agent.team), 0])
        local_target = agent.me.local(target - agent.me.location)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        # Use speed flip if far enough and have boost
        if magnitude(local_target) > 1200:
             agent.pop()
             # Target slightly to side of ball to hit center?
             agent.push(speed_flip(agent.ball.location))
             return

        if magnitude(local_target) < 650:
            agent.pop()
            agent.push(flip(agent.me.local(agent.foe_goal.location - agent.me.location)))

class speed_flip_kickoff():
    # Dedicated kickoff routine
    def __init__(self):
        self.step = 0

    def run(self, agent):
        if self.step == 0:
            agent.push(speed_flip(agent.ball.location))
            self.step = 1
        elif self.step == 1:
            # wait for speed flip to pop
            if not agent.me.airborne and distance(agent.me.location, agent.ball.location) < 800:
                agent.pop()
                agent.push(flip(agent.me.local(agent.ball.location - agent.me.location)))

class demo_hunt():
    def __init__(self, target_car):
        self.target_car = target_car
        self.start_time = -1

    def run(self, agent):
        if self.start_time == -1:
            self.start_time = agent.time

        elapsed = agent.time - self.start_time

        # Predict target location
        target_loc = self.target_car.location + self.target_car.velocity * 0.5
        local_target = agent.me.local(target_loc - agent.me.location)
        dist = magnitude(target_loc - agent.me.location)

        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        # Use speed flip if far
        if dist > 2000 and abs(atan2(local_target[1], local_target[0])) < 0.1:
            agent.push(speed_flip(target_loc))
            return

        # Abort if taking too long or target demolished
        if elapsed > 3.0 or self.target_car.demolished:
            agent.pop()

        if dist < 300:
            agent.pop() # Hit?

class fake_challenge():
    def __init__(self):
        self.step = 0
        self.start_time = -1

    def run(self, agent):
        if self.start_time == -1:
            self.start_time = agent.time

        elapsed = agent.time - self.start_time
        ball_loc = agent.ball.location

        if self.step == 0:
            # Drive aggressively at ball
            defaultPD(agent, agent.me.local(ball_loc - agent.me.location))
            defaultThrottle(agent, 2300)

            if elapsed > 0.5 or distance(agent.me.location, ball_loc) < 1000:
                self.step = 1

        elif self.step == 1:
            # Brake and turn away
            agent.controller.throttle = -1
            agent.controller.handbrake = True

            # Turn towards own goal/side
            turn_target = agent.friend_goal.location
            defaultPD(agent, agent.me.local(turn_target - agent.me.location))

            if elapsed > 1.0:
                agent.pop()
                agent.push(recovery())

class recovery():
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target


    def run(self, agent):
        if self.target is not None:
            local_target = agent.me.local(flatten(self.target - agent.me.location))
        else:
            local_target = agent.me.local(flatten(agent.me.velocity))

        defaultPD(agent, local_target)
        agent.controller.throttle = 1
        if not agent.me.airborne:
            agent.pop()


class short_shot():
    # This routine drives towards the ball and attempts to hit it towards a given target
    # It does not require ball prediction and kinda guesses at where the ball will be on its own
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball, distance = normalize(agent.ball.location - agent.me.location)
        ball_to_target, _ = normalize(self.target - agent.ball.location)

        relative_velocity = np.dot(car_to_ball, agent.me.velocity - agent.ball.velocity)
        if relative_velocity != 0.0:
            eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
        else:
            eta = 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        # left_vector = car_to_ball.cross((0, 0, 1))
        # right_vector = car_to_ball.cross((0, 0, -1))
        left_vector = np.cross(car_to_ball, np.array([0, 0, 1]))
        right_vector = np.cross(car_to_ball, np.array([0, 0, -1]))

        # target_vector = -ball_to_target.clamp(left_vector, right_vector)
        target_vector = -clamp(ball_to_target, left_vector, right_vector)

        final_target = agent.ball.location + (target_vector * (distance / 2))

        # Some adjustment to the final target to ensure we don't try to dirve through any goalposts to reach it
        #if abs(agent.me.location[1]) > 5120: final_target[0] = cap(final_target[0], -750, 750)
        if in_goal_area(agent):
            final_target[0] = cap(final_target[0], -750, 750)
            final_target[1] = cap(final_target[1], -5050, 5050)

        agent.line(final_target - np.array([0, 0, 100]), final_target + np.array([0, 0, 100]), [255, 255, 255])

        angles = defaultPD(agent, agent.me.local(final_target - agent.me.location))
        defaultThrottle(agent, 2300 if distance > 1600 else 2300 - cap(1600 * abs(angles[1]), 0, 2050))
        agent.controller.boost = False if agent.me.airborne or abs(angles[1]) > 0.3 else agent.controller.boost
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.pop()
            agent.push(flip(agent.me.local(car_to_ball)))


class wave_dash():
    def __init__(self):
        self.start = -1
        self.target = None
        self.landing_time = -1

    def run(self, agent):
        agent.controller.throttle = 1
        if self.start == -1:
            self.start = agent.time
            elapsed = 0
            self.target = agent.me.forward + agent.me.up
            self.target, _ = normalize(self.target)
        else:
            elapsed = agent.time - self.start
        agent.line(agent.me.location, agent.me.location + (self.target * 200))
        if elapsed < 0.05:
            agent.controller.jump = True
        elif elapsed < 0.15:
            agent.controller.jump = False
            # defaultPD(agent, self.target)
            up = agent.me.local(np.array([0, 0, 1]))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(self.target[2], self.target[0]),  # angle required to pitch towards target
                math.atan2(self.target[1], self.target[0]),  # angle required to yaw towards target
                math.atan2(up[1], up[2])]
            agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 4)

        elif agent.me.airborne and 18 < agent.me.location[2] < 45:
            agent.controller.jump = True
            agent.controller.pitch = -1
            agent.controller.handbrake = True
        elif not agent.me.airborne:
            if self.landing_time == -1:
                self.landing_time = agent.time
            if agent.time - self.landing_time < 0.18:
                agent.controller.handbrake = True
            else:
                agent.pop()
        if elapsed > 2:
            agent.pop()
            if agent.me.airborne:
                agent.push(recovery())


class boost_wave_dash():
    def __init__(self):
        self.start = -1
        self.target = None
        self.landing_time = -1

    def run(self, agent):
        agent.controller.throttle = 1
        if self.start == -1:
            self.start = agent.time
            elapsed = 0
            self.target = agent.me.forward - agent.me.up
            self.target, _ = normalize(self.target)
        else:
            elapsed = agent.time - self.start
        agent.line(agent.me.location, agent.me.location + (self.target * 200))
        if elapsed < 0.10:
            agent.controller.jump = True
            agent.controller.boost = True
            up = agent.me.local(np.array([0, 0, 1]))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(self.target[2], self.target[0]),  # angle required to pitch towards target
                math.atan2(self.target[1], self.target[0]),  # angle required to yaw towards target
                math.atan2(up[1], up[2])]
            agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 4)
        elif elapsed < 0.25:
            agent.controller.jump = False
            agent.controller.boost = True
            up = agent.me.local(np.array([0, 0, 1]))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(self.target[2], self.target[0]),  # angle required to pitch towards target
                math.atan2(self.target[1], self.target[0]),  # angle required to yaw towards target
                math.atan2(up[1], up[2])]
            agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 4)
        elif elapsed < 0.66:
            agent.controller.boost = True if agent.me.forward[2] < 0 else False
            self.target = agent.me.forward + agent.me.up
            up = agent.me.local(np.array([0, 0, 1]))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(self.target[2], self.target[0]),  # angle required to pitch towards target
                math.atan2(self.target[1], self.target[0]),  # angle required to yaw towards target
                math.atan2(up[1], up[2])]
            agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 4)
        elif agent.me.airborne and 18 < agent.me.location[2] < 45:
            agent.controller.jump = True
            agent.controller.pitch = -1
            agent.controller.handbrake = True
            agent.controller.boost = True
        elif not agent.me.airborne:
            agent.controller.boost = True
            if self.landing_time == -1:
                self.landing_time = agent.time
            if agent.time - self.landing_time < 0.10:
                agent.controller.handbrake = True
            else:
                agent.pop()
        if elapsed > 2:
            agent.pop()


class avoid_demo():
    def __init__(self, car):
        self.car = car
        self.direction = None
        self.start = -1

    def run(self, agent):
        botToTargetAngle = atan2(self.car.location[1] - agent.me.location[1],
                                 self.car.location[0] - agent.me.location[0])
        # yaw2 = atan2(agent.me.orientation[1][0], agent.me.orientation[0][0])
        # In Matrix3, orientation[1] is left, orientation[0] is forward?
        # car_object.orientation is [forward, left, up] columns?
        # car_object.__init__:
        # forward = np.array([CP * CY, CP * SY, SP])
        # self.orientation = np.column_stack((forward, left, up))
        # So agent.me.orientation[:, 0] is forward.
        # agent.me.forward is already defined as property.

        yaw2 = atan2(agent.me.forward[1], agent.me.forward[0])

        if botToTargetAngle + yaw2 < 0:
            self.direction = 1
        else:
            self.direction = -1
        if self.start == -1:
            self.start = agent.time
        elapsed = agent.time - self.start
        if elapsed < 0.2:
            agent.controller.jump = True
        elif elapsed < 0.25:
            agent.controller.jump = False
            agent.controller.roll = self.direction
        elif elapsed < 0.45:
            agent.controller.jump = True
            agent.controller.roll = self.direction
        elif not agent.me.airborne:
            agent.pop()
        elif elapsed > 1:
            agent.push(recovery())


class aerial():

    def __init__(self, ball_location: np.array, intercept_time: float, on_ground: bool, target: np.array = None):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.target = target
        self.jumping = on_ground
        self.time = -1
        self.jump_time = -1
        self.counter = 0

    def run(self, agent):
        agent.stopped_aerial = False
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        T = self.intercept_time - agent.time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * gravity * T ** 2
        vf = agent.me.velocity + gravity * T
        if self.jumping:
            if self.jump_time == -1:
                jump_elapsed = 0
                self.jump_time = agent.time
            else:
                jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed
            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            vf += agent.me.up * jump_speed
            xf += agent.me.up * jump_speed * (T - tau)

            if jump_elapsed < jump_max_duration:
                agent.controller.jump = True
            elif elapsed >= jump_max_duration and self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif elapsed < 0.3:
                agent.controller.jump = True
            else:
                self.jumping = jump_elapsed <= 0.3
        else:
            agent.controller.jump = 0

        delta_x = self.ball_location - xf
        direction, _ = normalize(delta_x)
        if magnitude(delta_x) > 50:
            defaultPD(agent, agent.me.local(delta_x))
        else:
            if self.target is not None:
                defaultPD(agent, agent.me.local(self.target))
            else:
                defaultPD(agent, agent.me.local(self.ball_location - agent.me.location))

        if jump_max_duration <= elapsed < 0.3 and self.counter == 3:
            agent.controller.roll = 0
            agent.controller.pitch = 0
            agent.controller.yaw = 0
            agent.controller.steer = 0

        # if agent.me.forward.angle3D(direction) < 0.3:
        if angle3D(agent.me.forward, direction) < 0.3:
            if magnitude(delta_x) > 50:
                agent.controller.boost = 1
                agent.controller.throttle = 0
            else:
                agent.controller.boost = 0
                agent.controller.throttle = cap(0.5 * throttle_accel * T ** 2, 0, 1)
        else:
            agent.controller.boost = 0
            agent.controller.throttle = 0

        if T <= 0 or not shot_valid(agent, self, threshold=150):
            agent.pop()
            agent.stopped_aerial = True
            #agent.push(recovery()))

    def is_viable(self, agent, time: float):
        T = self.intercept_time - time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * gravity * T ** 2
        vf = agent.me.velocity + gravity * T
        if not agent.me.airborne:
            vf += agent.me.up * (2 * jump_speed + jump_acc * jump_max_duration)
            xf += agent.me.up * (jump_speed * (2 * T - jump_max_duration) + jump_acc * (
                    T * jump_max_duration - 0.5 * jump_max_duration ** 2))

        delta_x = self.ball_location - xf
        f, _ = normalize(delta_x)
        # phi = f.angle3D(agent.me.forward)
        phi = angle3D(f, agent.me.forward)

        turn_time = 0.7 * (2 * math.sqrt(phi / 9))

        tau1 = turn_time * cap(1 - 0.3 / phi, 0, 1)
        required_acc = (2 * magnitude(delta_x)) / ((T - tau1) ** 2)
        ratio = required_acc / boost_accel
        tau2 = T - (T - tau1) * math.sqrt(1 - cap(ratio, 0, 1))
        velocity_estimate = vf + boost_accel * (tau2 - tau1) * f
        boos_estimate = (tau2 - tau1) * 30
        enough_boost = boos_estimate < 0.95 * agent.me.boost
        enough_time = abs(ratio) < 0.9
        return magnitude(velocity_estimate) < 0.9 * max_speed and enough_boost and enough_time

class air_dribble():
    def __init__(self, target=None):
        self.step = 0
        self.target = target

    def run(self, agent):
        ball_loc = agent.ball.location
        my_loc = agent.me.location
        dist_to_ball = distance(my_loc, ball_loc)

        # Step 0: Setup / Approach
        if self.step == 0:
            if agent.me.airborne:
                self.step = 2 # Already in air, go to carry
            elif dist_to_ball > 500:
                # Drive to ball
                agent.push(goto(ball_loc, urgent=True))
                # Note: push adds to stack, pop removes current.
                # Ideally we want to drive *then* check again.
                # But pushing a routine puts it on TOP.
                # So next tick, goto runs. When goto pops, we are back here?
                # No, air_dribble.run is called every tick if it's the active routine.
                # GoslingUtils style: routines don't persist state well if they push other routines.
                # Instead, we should control the car directly.
                agent.pop() # Remove goto if we pushed it

            # Simple approach logic: Drive under the ball
            target = np.array([ball_loc[0], ball_loc[1], 0])
            angles = defaultPD(agent, agent.me.local(target - my_loc))
            defaultThrottle(agent, 1400) # Controlled speed

            if dist_to_ball < 200:
                self.step = 1
                agent.controller.jump = True # Pop

        # Step 1: Lift
        elif self.step == 1:
            agent.controller.jump = False
            agent.controller.pitch = 1 # Tilt back
            if agent.me.location[2] > 50: # Airborne
                self.step = 2

        # Step 2: Carry
        elif self.step == 2:
            # Match ball velocity
            target = ball_loc + np.array([0, 0, -50]) # Aim slightly below ball

            # Use aerial control logic (simplified from aerial class)
            delta_x = target - my_loc
            direction, _ = normalize(delta_x)
            defaultPD(agent, agent.me.local(delta_x))

            # Feather boost if pointing at ball
            if angle3D(agent.me.forward, direction) < 0.5:
                agent.controller.boost = True
            else:
                agent.controller.boost = False

            if ball_loc[2] < 100: # Dropped
                agent.pop()

class wall_shot():
    def __init__(self):
        self.step = 0

    def run(self, agent):
        ball_loc = agent.ball.location
        my_loc = agent.me.location

        # Step 0: Drive up wall
        if self.step == 0:
            # Check if on wall (up vector is not vertical)
            if agent.me.up[2] < 0.5:
                # We are on wall
                self.step = 1
            else:
                # Drive to wall nearest ball
                wall_target = np.array([3500 * sign(ball_loc[0]), ball_loc[1], 0])
                defaultPD(agent, agent.me.local(wall_target - my_loc))
                defaultThrottle(agent, 2300)

        # Step 1: Aim and Jump
        elif self.step == 1:
            # Aim at ball
            defaultPD(agent, agent.me.local(ball_loc - my_loc))
            defaultThrottle(agent, 1400)

            if distance(my_loc, ball_loc) < 500:
                agent.controller.jump = True
                self.step = 2

        # Step 2: Aerial to ball
        elif self.step == 2:
            agent.controller.jump = False
            # Transition to aerial
            agent.pop()
            agent.push(aerial(ball_loc, agent.time + 0.5, False))
