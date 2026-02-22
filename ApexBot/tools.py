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

            i += 15 - cap(int(ball_velocity//150),0,13)

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
                                # Relaxed constraints for jump_shot
                                if ball_location[2] <= 300 and slope > -0.5: # was > 0.0
                                    hits[pair].append(routines.jump_shot(ball_location,intercept_time,best_shot_vector,slope))

                                # Relaxed constraints for aerial_shot
                                if ball_location[2] > 300 and ball_location[2] < 900 and slope > 0.5: # was < 600, slope > 1.0
                                    hits[pair].append(routines.aerial_shot(ball_location,intercept_time,best_shot_vector,slope))

                                # Aerial check
                                if ball_location[2] > 500: # was > 600
                                    shot = routines.aerial(ball_location - 92 * best_shot_vector, intercept_time, True,
                                                    target=best_shot_vector)
                                    if shot.is_viable(agent, agent.time) and should_aerial(agent, shot):
                                        hits[pair].append(shot)

                            elif backward_flag and ball_location[2] <= 280 and slope > 0.25:
                                hits[pair].append(routines.jump_shot(ball_location,intercept_time,best_shot_vector,slope,-1))
        else:
            i += 1
    return hits


def determine_shot(agent, target, targets, target_count, defensive=False, center=False):
    # This function is being deprecated in favor of strategy.py but kept for compatibility
    if magnitude(agent.ball.velocity) > 0:
        hits = find_hits(agent, targets)
        if len(hits):
            pick_the_fastest = []
            for i in range(1, 1 + target_count):
                if len(hits[str(i)]):
                    shot = hits[str(i)][0]
                    hit_location = shot.ball_location
                    hit_time = shot.intercept_time
                    time_delta = hit_time - agent.time
                    location_delta = distance(agent.me.location, hit_location)
                    avg_speed = location_delta / time_delta
                    pick_the_fastest.append(shot)
                    if (avg_speed < 700):
                        continue
                    if not defensive:
                        if len(agent.stack): agent.pop()
                        agent.push(shot)
                        if type(shot) == routines.aerial: agent.aerialing = True
                        return True
            if len(pick_the_fastest):
                pick_the_fastest.sort(key=lambda shot: shot.intercept_time)
                if len(agent.stack): agent.pop()
                agent.push(pick_the_fastest[0])
                if type(shot) == routines.aerial: agent.aerialing = True
                return defensive
    if center: return False
    if len(agent.stack): agent.pop()
    shot = routines.short_shot(target)
    agent.push(shot)
    return not center


def determine_follow_up_shot(agent, targets, target_count):
    if magnitude(agent.ball.velocity) > 0:
        hits = find_hits(agent, targets)
        if len(hits):
            for i in range(1, 1 + target_count):
                if len(hits[str(i)]):
                    for shot in hits[str(i)]:
                        if type(shot) != routines.aerial:
                            continue
                        else:
                            agent.aerialing = False
                            if len(agent.stack): agent.pop()
                            agent.push(shot)
                            return True
    return False


def should_aerial(agent, shot:routines.aerial):
    return True
