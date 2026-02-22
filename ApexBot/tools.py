import routines
from utils import *
import numpy as np

#This file is for strategic tools

def find_hits(agent,targets):
    hits = {name:[] for name in targets}
    struct = agent.get_ball_prediction_struct()

    #Begin looking at slices 0.25s into the future
    i = 15
    while i < struct.num_slices:
        intercept_time = struct.slices[i].game_seconds
        time_remaining = intercept_time - agent.time
        if time_remaining > 0:
            ball_location = np.array([struct.slices[i].physics.location.x, struct.slices[i].physics.location.y, struct.slices[i].physics.location.z])
            ball_velocity = np.linalg.norm(np.array([struct.slices[i].physics.velocity.x, struct.slices[i].physics.velocity.y, struct.slices[i].physics.velocity.z]))

            if abs(ball_location[1]) > 5250:
                break

            # More granular search for SSL precision
            i += 10 - cap(int(ball_velocity//150),0,8) # was 15, now 10 for finer steps

            car_to_ball = ball_location - agent.me.location
            direction, dist = normalize(car_to_ball)

            forward_angle = angle_between(direction, agent.me.forward)
            backward_angle = math.pi - forward_angle

            forward_time = time_remaining - (forward_angle * 0.318)
            backward_time = time_remaining - (backward_angle * 0.418)

            forward_flag = forward_time > 0.0 and (dist*1.05 / forward_time) < (2290 if agent.me.boost > dist/100 else 1400)
            backward_flag = dist < 1500 and backward_time > 0.0 and (dist*1.05 / backward_time) < 1200

            if forward_flag or backward_flag:
                for pair in targets:
                    left,right,swapped = post_correction(ball_location,targets[pair][0],targets[pair][1])
                    if not swapped:
                        left_vector, _ = normalize(left - ball_location)
                        right_vector, _ = normalize(right - ball_location)
                        best_shot_vector = clamp(direction, left_vector, right_vector)

                        if in_field(ball_location - (200*best_shot_vector),1):
                            slope = find_slope(best_shot_vector,car_to_ball)
                            if forward_flag:
                                # Highly permissive constraints, let the brain decide utility
                                if ball_location[2] <= 300:
                                    hits[pair].append(routines.jump_shot(ball_location,intercept_time,best_shot_vector,slope))

                                if ball_location[2] > 200:
                                    hits[pair].append(routines.aerial_shot(ball_location,intercept_time,best_shot_vector,slope))

                                if ball_location[2] > 400:
                                    shot = routines.aerial(ball_location - 92 * best_shot_vector, intercept_time, True,
                                                    target=best_shot_vector)
                                    if shot.is_viable(agent, agent.time) and should_aerial(agent, shot):
                                        hits[pair].append(shot)

                            elif backward_flag and ball_location[2] <= 280:
                                hits[pair].append(routines.jump_shot(ball_location,intercept_time,best_shot_vector,slope,-1))
        else:
            i += 1
    return hits

def find_wall_hits(agent, targets):
    # Specialized detection for wall play
    hits = []

    # Is ball near wall?
    ball_loc = agent.ball.location
    if abs(ball_loc[0]) > 3000 or abs(ball_loc[1]) > 4000:
        # Are we close enough to make a play?
        if distance(agent.me.location, ball_loc) < 2000:
             # Basic physics check: is ball high enough to drive under?
             if ball_loc[2] > 100:
                 hits.append(routines.wall_shot())

    return hits

def determine_shot(agent, target, targets, target_count, defensive=False, center=False):
    # Deprecated by strategy.py
    pass

def determine_follow_up_shot(agent, targets, target_count):
    # Deprecated by strategy.py
    pass

def should_aerial(agent, shot:routines.aerial):
    return True
