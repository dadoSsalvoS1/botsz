from routines import *
from utils import *
import numpy as np

def find_hits(agent, targets):
    hits = {name: [] for name in targets}
    struct = agent.get_ball_prediction_struct()

    i = 15
    while i < struct.num_slices:
        slice_obj = struct.slices[i]
        intercept_time = slice_obj.game_seconds
        time_remaining = intercept_time - agent.time

        if time_remaining > 0:
            ball_location = np.array([slice_obj.physics.location.x, slice_obj.physics.location.y, slice_obj.physics.location.z])
            ball_velocity = np.array([slice_obj.physics.velocity.x, slice_obj.physics.velocity.y, slice_obj.physics.velocity.z])
            ball_speed = np.linalg.norm(ball_velocity)

            # Stop if goal
            if abs(ball_location[1]) > 5250:
                break

            # Backboard Read Logic
            # If ball is high and deep, and velocity Y is flipping or small, it might be a bounce.
            # Simplified: If ball is near backboard (y > 5000 or y < -5000) and z > 300.
            # We want to catch the rebound.
            # Future slice check: If slice i+10 has reversed Y velocity?
            # Let's just look for aerials regardless of bounce, but maybe extend the search time?

            # Double Tap: If we find a hit that requires a wall read, we queue it.
            # For now, standard aerial search will find the rebound if we look far enough ahead.

            i += 15 - cap(int(ball_speed // 150), 0, 13)

            car_to_ball = ball_location - agent.me.location
            distance = np.linalg.norm(car_to_ball)
            if distance != 0:
                direction = car_to_ball / distance
            else:
                direction = np.array([1, 0, 0])

            forward_angle = math.acos(cap(np.dot(direction, agent.me.forward), -1, 1))
            backward_angle = math.pi - forward_angle

            forward_time = time_remaining - (forward_angle * 0.318)
            backward_time = time_remaining - (backward_angle * 0.418)

            forward_flag = forward_time > 0.0 and (distance * 1.05 / forward_time) < (2290 if agent.me.boost > distance/100 else 1400)
            backward_flag = distance < 1500 and backward_time > 0.0 and (distance * 1.05 / backward_time) < 1200

            if forward_flag or backward_flag:
                for pair in targets:
                    left, right, swapped = post_correction(ball_location, targets[pair][0], targets[pair][1])
                    if not swapped:
                        left_vector = (left - ball_location)
                        norm_left = np.linalg.norm(left_vector)
                        if norm_left != 0: left_vector /= norm_left

                        right_vector = (right - ball_location)
                        norm_right = np.linalg.norm(right_vector)
                        if norm_right != 0: right_vector /= norm_right

                        best_shot_vector = (left + right) / 2 - ball_location
                        norm_best = np.linalg.norm(best_shot_vector)
                        if norm_best != 0: best_shot_vector /= norm_best

                        if in_field(ball_location - (200 * best_shot_vector), 1):
                            slope = find_slope(best_shot_vector, car_to_ball)
                            if forward_flag:
                                if ball_location[2] <= 300 and slope > 0.0:
                                    hits[pair].append(jump_shot(ball_location, intercept_time, best_shot_vector, slope))
                                if ball_location[2] > 300 and ball_location[2] < 600 and slope > 1.0 and (ball_location[2]-250) * 0.14 < agent.me.boost:
                                     hits[pair].append(aerial_shot(ball_location, intercept_time, best_shot_vector, slope))
                                if ball_location[2] > 600:
                                    # Fast Aerial logic handles Z > 600
                                    shot = aerial(ball_location - 92 * best_shot_vector, intercept_time, True, target=best_shot_vector)
                                    if shot.is_viable(agent, agent.time):
                                        hits[pair].append(shot)
                            elif backward_flag and ball_location[2] <= 280 and slope > 0.25:
                                hits[pair].append(jump_shot(ball_location, intercept_time, best_shot_vector, slope, -1))
        else:
            i += 1

    return hits

def determine_shot(agent, target, targets, target_count, defensive=False, center=False):
    ball_speed = np.linalg.norm(agent.ball.velocity)
    if ball_speed > 0:
        hits = find_hits(agent, targets)
        if len(hits):
            pick_the_fastest = []
            for i in range(1, 1 + target_count):
                key = str(i)
                if key in hits and len(hits[key]):
                    shot = hits[key][0]
                    pick_the_fastest.append(shot)

            if len(pick_the_fastest):
                # Sort by intercept time
                pick_the_fastest.sort(key=lambda s: s.intercept_time)

                # Check if we should air dribble instead?
                # If the ball is high and we are close, maybe air dribble?
                best_shot = pick_the_fastest[0]

                # Air Dribble check:
                # If we are close to ball, ball is high (> 400), and moving slowly?
                # This is a simple heuristic.
                dist_to_ball = np.linalg.norm(agent.ball.location - agent.me.location)
                if agent.ball.location[2] > 500 and dist_to_ball < 500 and agent.me.boost > 50:
                    # Can we air dribble?
                    # agent.push(air_dribble())
                    # return True
                    # Let's stick to aerial for now as it's more reliable for scoring.
                    pass

                if len(agent.stack): agent.pop()
                agent.push(best_shot)
                return True

    if center: return False

    if len(agent.stack): agent.pop()
    shot = short_shot(target)
    agent.push(shot)
    return not center
