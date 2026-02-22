from utils import *
from routines import *
from tools import *
import numpy as np
import math

# Constants
gravity = 650
max_speed = 2300
boost_accel = 1060
throttle_accel = 100
boost_per_second = 30
jump_speed = 291.667
jump_acc = 1458.3333
jump_min_duration = 0.025
jump_max_duration = 0.2

class atba:
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

class flip:
    def __init__(self, vector, cancel=False):
        norm = np.linalg.norm(vector)
        if norm != 0:
            self.vector = vector / norm
        else:
            self.vector = np.array([1, 0, 0]) # Default forward

        self.pitch = abs(self.vector[0]) * -sign(self.vector[0])
        self.yaw = abs(self.vector[1]) * sign(self.vector[1])
        self.cancel = cancel
        self.time = -1
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

class recovery:
    def __init__(self, target=None):
        self.target = target

    def run(self, agent):
        if self.target is not None:
            local_target = agent.me.local(self.target - agent.me.location)
        else:
            local_target = agent.me.local(agent.me.velocity)
        defaultPD(agent, local_target)
        agent.controller.throttle = 1
        if not agent.me.airborne:
            agent.pop()

class goto:
    def __init__(self, target, vector=None, direction=1, urgent=False):
        self.target = target
        self.vector = vector
        self.direction = direction
        self.urgent = urgent

    def run(self, agent):
        car_to_target = self.target - agent.me.location
        distance_remaining = np.linalg.norm(car_to_target)
        agent.line(self.target - np.array([0, 0, 500]), self.target + np.array([0, 0, 500]), [255, 0, 255])

        if self.vector is not None:
            cross_z = np.cross(self.vector, np.array([0,0,1]))
            side_of_vector = sign(np.dot(cross_z, car_to_target))
            car_to_target_perp = np.cross(car_to_target, np.array([0, 0, side_of_vector]))
            norm_perp = np.linalg.norm(car_to_target_perp)
            if norm_perp != 0:
                car_to_target_perp = car_to_target_perp / norm_perp
            else:
                car_to_target_perp = np.zeros(3)

            if distance_remaining != 0:
                v1 = car_to_target / distance_remaining
            else:
                v1 = np.array([1, 0, 0])

            v2 = self.vector
            angle = math.acos(cap(np.dot(v1, v2), -1, 1))
            adjustment = angle * distance_remaining / 3.14
            final_target = self.target + (car_to_target_perp * adjustment)
        else:
            final_target = self.target

        if in_goal_area(agent):
             final_target[0] = cap(final_target[0], -750, 750)
             final_target[1] = cap(final_target[1], -5050, 5050)

        local_target = agent.me.local(final_target - agent.me.location)
        angles = defaultPD(agent, local_target, self.direction)
        defaultThrottle(agent, 2300, self.direction)

        agent.controller.boost = True if self.urgent and distance_remaining > 1500 else False
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake
        velocity = 1 + np.linalg.norm(agent.me.velocity)

        if distance_remaining < 350:
            agent.pop()
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and distance_remaining / velocity > 2.0:
            if agent.me.up[2] < 0.9 or agent.me.airborne:
                agent.push(flip(local_target))
            elif abs(agent.controller.yaw) < 0.2:
                 pass
        elif abs(angles[1]) > 2.8 and velocity < 200:
            agent.push(flip(local_target, True))
        elif agent.me.airborne:
            agent.push(recovery(self.target))

class short_shot:
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball = agent.ball.location - agent.me.location
        distance = np.linalg.norm(car_to_ball)
        if distance != 0:
            car_to_ball_norm = car_to_ball / distance
        else:
            car_to_ball_norm = np.array([1, 0, 0])

        ball_to_target = (self.target - agent.ball.location)
        b2t_dist = np.linalg.norm(ball_to_target)
        if b2t_dist != 0:
            ball_to_target_norm = ball_to_target / b2t_dist
        else:
            ball_to_target_norm = np.array([1, 0, 0])

        relative_velocity = np.dot(car_to_ball_norm, agent.me.velocity - agent.ball.velocity)
        if relative_velocity != 0.0:
            eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
        else:
            eta = 1.5

        # Approach Logic: Aim for the opposite side of the ball from the target
        # Add offset to ensure we hit the correct spot on the ball
        offset_distance = 100 # Ball radius roughly

        # We want to be behind the ball relative to the target
        ideal_pos = agent.ball.location - (ball_to_target_norm * offset_distance)

        # If we are far, aim for that ideal pos. If close, just hit the ball.
        if distance > 300:
            final_target = ideal_pos
        else:
            final_target = agent.ball.location

        # Correction for approach angle
        # If we are too far off the line, drift out
        car_to_ideal = ideal_pos - agent.me.location
        approach_angle = math.acos(cap(np.dot(car_to_ball_norm, ball_to_target_norm), -1, 1))

        # If angle is bad (> 45 deg), maybe circle?
        # For now, simple PD will try to turn.

        if in_goal_area(agent):
             final_target[0] = cap(final_target[0], -750, 750)
             final_target[1] = cap(final_target[1], -5050, 5050)

        agent.line(final_target - np.array([0, 0, 100]), final_target + np.array([0, 0, 100]), [255, 255, 255])
        angles = defaultPD(agent, agent.me.local(final_target - agent.me.location))

        # Throttle logic
        speed_req = 2300
        if distance < 1600:
            speed_req = 2300 - cap(1600 * abs(angles[1]), 0, 2050)

        defaultThrottle(agent, speed_req)

        agent.controller.boost = False if agent.me.airborne or abs(angles[1]) > 0.3 else agent.controller.boost
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        # Hit logic
        if abs(angles[1]) < 0.1 and (eta < 0.45 or distance < 200):
            agent.pop()
            # If ball is slightly in air, jump
            if agent.ball.location[2] > 100:
                 agent.push(flip(agent.me.local(car_to_ball))) # This triggers jump
            else:
                 agent.push(flip(agent.me.local(car_to_ball)))

class goto_boost:
    def __init__(self, boost, target=None):
        self.boost = boost
        self.target = target
        self.start = -1

    def run(self, agent):
        if self.start == -1:
            self.start = agent.time
        elapsed = agent.time - self.start
        car_to_boost = self.boost.location - agent.me.location
        distance_remaining = np.linalg.norm(car_to_boost)
        agent.line(self.boost.location - np.array([0, 0, 500]), self.boost.location + np.array([0, 0, 500]), [0, 255, 0])

        if self.target is not None:
            vector = (self.target - self.boost.location)
            norm_v = np.linalg.norm(vector)
            if norm_v != 0:
                vector = vector / norm_v
            else:
                vector = np.array([1, 0, 0])

            cross_z = np.cross(vector, np.array([0,0,1]))
            side_of_vector = sign(np.dot(cross_z, car_to_boost))
            car_to_boost_perp = np.cross(car_to_boost, np.array([0,0,side_of_vector]))
            norm_perp = np.linalg.norm(car_to_boost_perp)
            if norm_perp != 0:
                car_to_boost_perp = car_to_boost_perp / norm_perp
            else:
                car_to_boost_perp = np.zeros(3)

            if distance_remaining != 0:
                v1 = car_to_boost / distance_remaining
            else:
                v1 = np.array([1, 0, 0])

            angle = math.acos(cap(np.dot(v1, vector), -1, 1))
            adjustment = angle * distance_remaining / 3.14
            final_target = self.boost.location + (car_to_boost_perp * adjustment)
        else:
            final_target = np.array(self.boost.location)

        if in_goal_area(agent):
             final_target[0] = cap(final_target[0], -750, 750)
             final_target[1] = cap(final_target[1], -5050, 5050)

        local_target = agent.me.local(final_target - agent.me.location)
        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)
        agent.controller.boost = self.boost.large if abs(angles[1]) < 0.3 else False
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        if elapsed > 3 or self.boost.active == False or agent.me.boost >= 99.0 or distance_remaining < 350:
            agent.pop()
        elif agent.me.airborne:
            agent.push(recovery(self.target))

# --- Advanced Routines ---

class jump_shot:
    def __init__(self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - (self.shot_vector * 173)
        self.ratio = ratio
        self.direction = direction
        self.speed_desired = speed
        self.jump_threshold = 400
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        cross_z = np.cross(self.shot_vector, np.array([0,0,1]))
        side_of_shot = sign(np.dot(cross_z, car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = np.cross(car_to_dodge_point, np.array([0, 0, side_of_shot]))
        distance_remaining = np.linalg.norm(car_to_dodge_point)

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650)
        local_acceleration_required = agent.me.local(acceleration_required)

        if distance_remaining != 0:
            v1 = car_to_dodge_point / distance_remaining
        else:
            v1 = np.array([1, 0, 0])

        angle = math.acos(cap(np.dot(v1, self.shot_vector), -1, 1))
        adjustment = angle * distance_remaining / 2.0
        adjustment *= (cap(self.jump_threshold - (acceleration_required[2]), 0.0, self.jump_threshold) / self.jump_threshold)

        norm_perp = np.linalg.norm(car_to_dodge_perp)
        perp_vec = car_to_dodge_perp / norm_perp if norm_perp != 0 else np.zeros(3)

        final_target = self.dodge_point + (perp_vec * adjustment if not self.jumping else 0) + np.array([0, 0, 50])

        if in_goal_area(agent):
             final_target[0] = cap(final_target[0], -750, 750)
             final_target[1] = cap(final_target[1], -5050, 5050)

        local_final_target = agent.me.local(final_target - agent.me.location)
        agent.line(agent.me.location, self.dodge_point)
        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 and self.direction == 1 else agent.controller.handbrake

        if not self.jumping:
            if raw_time_remaining <= 0.0 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif local_acceleration_required[2] > self.jump_threshold and local_acceleration_required[2] > np.linalg.norm(local_acceleration_required[:2]):
                self.jumping = True
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 150)) or raw_time_remaining <= -0.9 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and local_acceleration_required[2] > 0.0 and raw_time_remaining > 0.083:
                agent.controller.jump = True
            elif self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -sign(vector[0])
                    self.y = abs(vector[1]) * sign(vector[1]) * self.direction
                    self.dodging = True
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0

class aerial_shot:
    def __init__(self, ball_location, intercept_time, shot_vector, ratio):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.intercept = self.ball_location - (self.shot_vector * 110)
        self.jump_threshold = 600
        self.jump_time = 0
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.01, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        cross_z = np.cross(self.shot_vector, np.array([0,0,1]))
        side_of_shot = sign(np.dot(cross_z, car_to_ball))

        car_to_intercept = self.intercept - agent.me.location
        car_to_intercept_perp = np.cross(car_to_intercept, np.array([0, 0, side_of_shot]))
        distance_remaining = np.linalg.norm(car_to_intercept[:2]) # Flattened magnitude

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(self.intercept, agent.me, time_remaining, 0 if self.jump_time == 0 else 325)
        local_acceleration_required = agent.me.local(acceleration_required)

        norm_intercept = np.linalg.norm(car_to_intercept)
        if norm_intercept != 0:
            v1 = car_to_intercept / norm_intercept
        else:
            v1 = np.array([1, 0, 0])

        angle = math.acos(cap(np.dot(v1, self.shot_vector), -1, 1))
        adjustment = angle * distance_remaining / 1.57
        adjustment *= (cap(self.jump_threshold - (acceleration_required[2]), 0.0, self.jump_threshold) / self.jump_threshold)

        norm_perp = np.linalg.norm(car_to_intercept_perp)
        perp_vec = car_to_intercept_perp / norm_perp if norm_perp != 0 else np.zeros(3)

        final_target = self.intercept + (perp_vec * adjustment if self.jump_time == 0 else 0)

        if in_goal_area(agent):
             final_target[0] = cap(final_target[0], -750, 750)
             final_target[1] = cap(final_target[1], -5050, 5050)

        local_final_target = agent.me.local(final_target - agent.me.location)
        angles = defaultPD(agent, local_final_target)

        if self.jump_time == 0:
            defaultThrottle(agent, speed_required)
            agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
            agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake
            if acceleration_required[2] > self.jump_threshold:
                self.jump_time = agent.time
        else:
            time_since_jump = agent.time - self.jump_time
            if agent.me.airborne and np.linalg.norm(local_acceleration_required) * time_remaining > 100:
                angles = defaultPD(agent, local_acceleration_required)
                if abs(angles[0]) + abs(angles[1]) < 0.5:
                    agent.controller.boost = True
            if self.counter == 0 and (time_since_jump <= 0.2 and local_acceleration_required[2] > 0):
                agent.controller.jump = True
            elif time_since_jump > 0.2 and self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif local_acceleration_required[2] > 300 and self.counter == 3:
                agent.controller.jump = True
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                self.counter += 1

        if raw_time_remaining < -0.25 or not shot_valid(agent, self):
            agent.pop()
            agent.push(recovery())

class aerial:
    def __init__(self, ball_location, intercept_time, on_ground, target=None):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.target = target
        self.jumping = on_ground
        self.time = -1
        self.jump_time = -1
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        T = self.intercept_time - agent.time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * np.array([0,0,-650]) * T ** 2
        vf = agent.me.velocity + np.array([0,0,-650]) * T

        # FAST AERIAL LOGIC (Boost while double jumping)
        if self.jumping:
            if self.jump_time == -1:
                jump_elapsed = 0
                self.jump_time = agent.time
            else:
                jump_elapsed = agent.time - self.jump_time

            # Simple jump physics model
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
                # Hold pitch back slightly for fast aerial takeoff
                agent.controller.pitch = 1.0
                # Boost if aiming high
                agent.controller.boost = True
            elif elapsed >= jump_max_duration and self.counter < 3:
                agent.controller.jump = False # Release
                agent.controller.pitch = 0
                agent.controller.boost = True
                self.counter += 1
            elif elapsed < 0.3:
                agent.controller.jump = True # Second jump
                agent.controller.pitch = 0
                agent.controller.roll = 0
                agent.controller.yaw = 0
                agent.controller.boost = True
            else:
                self.jumping = jump_elapsed <= 0.3 # Done jumping
        else:
            agent.controller.jump = False

        delta_x = self.ball_location - xf
        norm_delta = np.linalg.norm(delta_x)
        if norm_delta != 0:
            direction = delta_x / norm_delta
        else:
            direction = np.array([1, 0, 0])

        if norm_delta > 50:
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

        angle_diff = math.acos(cap(np.dot(agent.me.forward, direction), -1, 1))

        if angle_diff < 0.3:
            if norm_delta > 50:
                agent.controller.boost = True
                agent.controller.throttle = 0
            else:
                agent.controller.boost = False
                agent.controller.throttle = cap(0.5 * throttle_accel * T ** 2, 0, 1)
        else:
            agent.controller.boost = False
            agent.controller.throttle = 0

        # Force boost during fast aerial takeoff
        if self.jumping and (self.jump_time != -1 and agent.time - self.jump_time < jump_max_duration + 0.1):
             agent.controller.boost = True

        if T <= 0 or not shot_valid(agent, self, threshold=150):
            agent.pop()
            agent.push(recovery())

    def is_viable(self, agent, time):
        # Adjusted for fast aerial capability (more boost/accel)
        T = self.intercept_time - time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * np.array([0,0,-650]) * T ** 2
        vf = agent.me.velocity + np.array([0,0,-650]) * T

        if not agent.me.airborne:
            # Fast aerial adds more height/velocity
            vf += agent.me.up * (2 * jump_speed + jump_acc * jump_max_duration)
            xf += agent.me.up * (jump_speed * (2 * T - jump_max_duration) + jump_acc * (
                    T * jump_max_duration - 0.5 * jump_max_duration ** 2))

        delta_x = self.ball_location - xf
        norm_delta = np.linalg.norm(delta_x)
        if norm_delta != 0:
            f = delta_x / norm_delta
        else:
            f = np.array([1, 0, 0])

        phi = math.acos(cap(np.dot(f, agent.me.forward), -1, 1))
        turn_time = 0.7 * (2 * math.sqrt(phi / 9))

        tau1 = turn_time * cap(1 - 0.3 / (phi if phi!=0 else 0.001), 0, 1)
        required_acc = (2 * norm_delta) / ((T - tau1) ** 2)
        ratio = required_acc / boost_accel
        tau2 = T - (T - tau1) * math.sqrt(1 - cap(ratio, 0, 1))
        velocity_estimate = vf + boost_accel * (tau2 - tau1) * f
        boost_estimate = (tau2 - tau1) * 30
        enough_boost = boost_estimate < 0.95 * agent.me.boost
        enough_time = abs(ratio) < 0.9
        return np.linalg.norm(velocity_estimate) < 0.9 * max_speed and enough_boost and enough_time

class speed_flip:
    def __init__(self, target):
        self.target = target
        self.start_time = -1
        self.jump_timer = 0
        self.phase = 0

    def run(self, agent):
        if self.start_time == -1:
            self.start_time = agent.time

        local_target = agent.me.local(self.target - agent.me.location)
        defaultPD(agent, local_target)

        if self.phase == 0:
            defaultThrottle(agent, 2300)
            if np.linalg.norm(agent.me.velocity) > 500: # Tune speed
                self.phase = 1
                self.jump_timer = agent.time
        elif self.phase == 1: # Jump
            defaultThrottle(agent, 2300)
            jump_elapsed = agent.time - self.jump_timer
            if jump_elapsed < 0.05:
                agent.controller.jump = True
            elif jump_elapsed < 0.1:
                agent.controller.jump = False
            else:
                self.phase = 2
                self.jump_timer = agent.time
        elif self.phase == 2: # Dodge
            defaultThrottle(agent, 2300)
            agent.controller.jump = True
            agent.controller.pitch = -1
            # Slight diagonal
            angle = math.atan2(local_target[1], local_target[0])
            agent.controller.yaw = sign(angle) * 0.2 if abs(angle) > 0.05 else 0
            agent.controller.roll = 0
            self.phase = 3
            self.jump_timer = agent.time
        elif self.phase == 3: # Cancel
            defaultThrottle(agent, 2300)
            cancel_elapsed = agent.time - self.jump_timer
            if cancel_elapsed < 0.05: # Hold dodge for a split second
                agent.controller.jump = True
                agent.controller.pitch = -1
            elif cancel_elapsed < 0.5: # Cancel
                agent.controller.jump = False
                agent.controller.pitch = 1 # Pull back hard
                agent.controller.roll = 0 # Hold roll?
                agent.controller.yaw = 0
            else:
                agent.pop()
                agent.push(recovery())

class wave_dash:
    def __init__(self):
        self.start = -1
        self.landing_time = -1

    def run(self, agent):
        agent.controller.throttle = 1
        if self.start == -1:
            self.start = agent.time
            elapsed = 0
        else:
            elapsed = agent.time - self.start

        if elapsed < 0.05:
            agent.controller.jump = True
        elif elapsed < 0.15:
            agent.controller.jump = False
            # pitch down
            agent.controller.pitch = 1
        elif agent.me.airborne and agent.me.location[2] < 45: # Close to ground
            agent.controller.jump = True
            agent.controller.pitch = -1 # Forward dodge
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

class air_dribble:
    def __init__(self):
        pass

    def run(self, agent):
        if not agent.me.airborne:
            agent.pop()
            return

        # Aim slightly below the ball to prop it up
        target = agent.ball.location + np.array([0, 0, -50])
        local_target = agent.me.local(target - agent.me.location)
        defaultPD(agent, local_target)

        car_speed = np.linalg.norm(agent.me.velocity)
        ball_speed = np.linalg.norm(agent.ball.velocity)

        # Distance check
        dist = np.linalg.norm(agent.ball.location - agent.me.location)

        if dist > 1000: # Lost control
            agent.pop()
            agent.push(recovery())
            return

        # Throttle/Boost control
        # If we are behind the ball and moving slower, boost/throttle
        forward_dot = np.dot(agent.me.forward, (agent.ball.location - agent.me.location) / dist)

        if forward_dot > 0.8:
            if car_speed < ball_speed + 50:
                agent.controller.throttle = 1.0
                agent.controller.boost = True
            else:
                agent.controller.throttle = 0.0
                agent.controller.boost = False
        else:
            agent.controller.throttle = 0.5 # Maintain adjustment

        # Vertical control - keep altitude relative to ball
        if agent.me.location[2] < agent.ball.location[2] - 100:
            agent.controller.jump = True # Try to jump/dodge up? No, aerial logic.
            agent.controller.pitch = 1 # Pull back
            agent.controller.boost = True

        if agent.me.location[2] < 100: # Landed
            agent.pop()
            agent.push(recovery())

class dribble:
    def __init__(self):
        pass

    def run(self, agent):
        ball_xy = agent.ball.location[:2]
        ball_vel_xy = agent.ball.velocity[:2]
        target_loc = agent.ball.location
        if target_loc[2] > 150:
            agent.pop()
            agent.push(atba())
            return

        local_target = agent.me.local(target_loc - agent.me.location)
        defaultPD(agent, local_target)
        dist = np.linalg.norm(target_loc - agent.me.location)
        if dist > 200:
             defaultThrottle(agent, 2300)
        else:
             ball_speed = np.linalg.norm(ball_vel_xy)
             defaultThrottle(agent, ball_speed)

class wall_shot:
    def __init__(self, target):
        self.target = target
        self.jumping = False

    def run(self, agent):
        # Basic wall shot logic
        # 1. Drive up wall matching ball X/Y
        # 2. When close to ball Z, jump off wall towards target

        # Are we on the wall?
        # Wall is X +/- 4096 or Y +/- 5120
        # If agent.me.location[2] > 50?

        if not agent.me.airborne and agent.me.location[2] < 20:
            # Drive to wall
            # Find closest wall point
            pass
            # This complex. For now, simple fallback.
            agent.pop()
            agent.push(short_shot(self.target))
            return

        # Aim at ball
        local_target = agent.me.local(agent.ball.location - agent.me.location)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        dist = np.linalg.norm(agent.ball.location - agent.me.location)

        if dist < 300:
            agent.controller.jump = True
            agent.controller.pitch = -1 # Flip into it?
            if dist < 150:
                 agent.pop()
                 agent.push(recovery())
