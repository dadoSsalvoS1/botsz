from routines import *
from utils import *
import numpy as np

#This file is for strategic tools

def find_hits(agent, targets):
    hits = {name:[] for name in targets}
    struct = agent.get_ball_prediction_struct()

    #Begin looking at slices 0.25s into the future
    i = 15
    while i < struct.num_slices:
        #Gather some data about the slice
        intercept_time = struct.slices[i].game_seconds
        time_remaining = intercept_time - agent.time
        if time_remaining > 0:
            ball_location = np.array([struct.slices[i].physics.location.x, struct.slices[i].physics.location.y, struct.slices[i].physics.location.z])
            ball_velocity = np.array([struct.slices[i].physics.velocity.x, struct.slices[i].physics.velocity.y, struct.slices[i].physics.velocity.z])
            ball_velocity_mag = np.linalg.norm(ball_velocity)

            if abs(ball_location[1]) > 5250:
                break #abandon search if ball is scored at/after this point

            #determine the next slice we will look at, based on ball velocity (slower ball needs fewer slices)
            i += 15 - cap(int(ball_velocity_mag//150), 0, 13)

            car_to_ball = ball_location - agent.me.location
            direction = normalize(car_to_ball)
            distance = np.linalg.norm(car_to_ball)

            #How far the car must turn in order to face the ball, for forward and reverse
            forward_angle = angle3D(direction, agent.me.forward)
            backward_angle = math.pi - forward_angle

            #Accounting for the average time it takes to turn and face the ball
            forward_time = time_remaining - (forward_angle * 0.318)
            backward_time = time_remaining - (backward_angle * 0.418)

            #If the car only had to drive in a straight line, we ensure it has enough time to reach the ball
            forward_flag = forward_time > 0.0 and (distance*1.05 / forward_time) < (2290 if agent.me.boost > distance/100 else 1400)
            backward_flag = distance < 1500 and backward_time > 0.0 and (distance*1.05 / backward_time) < 1200

            if forward_flag or backward_flag:
                for pair in targets:
                    left_target = targets[pair][0]
                    right_target = targets[pair][1]

                    left, right, swapped = post_correction(ball_location, left_target, right_target)
                    if not swapped:
                        left_vector = normalize(left - ball_location)
                        right_vector = normalize(right - ball_location)
                        # Clamp direction between left and right vectors
                        # We need to map 3D vectors to 2D for clamping usually, but let's assume z is handled
                        best_shot_vector = clamp_vector(direction, left_vector, right_vector)

                        if in_field(ball_location - (200*best_shot_vector), 1):
                            slope = find_slope(best_shot_vector, car_to_ball)
                            if forward_flag:
                                if ball_location[2] <= 300 and slope > 0.0:
                                    hits[pair].append(jump_shot(ball_location, intercept_time, best_shot_vector, slope))
                                if ball_location[2] > 300 and ball_location[2] < 600 and slope > 1.0 and (ball_location[2]-250) * 0.14 > agent.me.boost:
                                    hits[pair].append(aerial_shot(ball_location, intercept_time, best_shot_vector, slope))
                                if ball_location[2] > 600:
                                    shot = aerial(ball_location - 92 * best_shot_vector, intercept_time, True, target=best_shot_vector)
                                    if shot.is_viable(agent, agent.time) and should_aerial(agent, shot):
                                        hits[pair].append(shot)
                            elif backward_flag and ball_location[2] <= 280 and slope > 0.25:
                                hits[pair].append(jump_shot(ball_location, intercept_time, best_shot_vector, slope, -1))
        else:
            i += 1
    return hits


def determine_shot(agent, target, targets, target_count, defensive=False, center=False):
    if np.linalg.norm(agent.ball.velocity) > 0:
        hits = find_hits(agent, targets)
        if len(hits):
            pick_the_fastest = []
            for i in range(1, 1 + target_count):
                key = str(i)
                if len(hits[key]):
                    shot = hits[key][0]
                    hit_location = shot.ball_location
                    hit_time = shot.intercept_time
                    time_delta = hit_time - agent.time
                    location_delta = np.linalg.norm(agent.me.location - hit_location)
                    avg_speed = location_delta / time_delta

                    pick_the_fastest.append(shot)
                    if (avg_speed < 700):
                        continue

                    if not defensive:
                        if len(agent.stack): agent.pop()
                        agent.push(shot)
                        if type(shot) == aerial: agent.aerialing = True
                        return True
            if len(pick_the_fastest):
                pick_the_fastest.sort(key=lambda shot: shot.intercept_time)
                if len(agent.stack): agent.pop()
                agent.push(pick_the_fastest[0])
                if type(shot) == aerial: agent.aerialing = True
                return True
    if center: return False
    if len(agent.stack): agent.pop()
    shot = short_shot(target)
    agent.push(shot)
    return not center


def determine_follow_up_shot(agent, targets, target_count):
    if np.linalg.norm(agent.ball.velocity) > 0:
        hits = find_hits(agent, targets)
        if len(hits):
            for i in range(1, 1 + target_count):
                key = str(i)
                if len(hits[key]):
                    for shot in hits[key]:
                        if type(shot) != aerial:
                            continue
                        else:
                            agent.aerialing = False
                            if len(agent.stack): agent.pop()
                            agent.push(shot)
                            return True
    return False


def should_aerial(agent, shot:aerial):
    return agent.me.boost > 30

def is_air_dribble_viable(agent):
    dist = np.linalg.norm(agent.ball.location - agent.me.location)
    if agent.ball.location[2] > 200 and dist < 1000 and agent.me.boost > 50:
        return True
    return False

# --- NEW STRATEGIC ANALYSIS FUNCTIONS ---

def intercept_time(car, ball_prediction):
    car_to_ball = np.linalg.norm(np.array([ball_prediction.slices[0].physics.location.x, ball_prediction.slices[0].physics.location.y, ball_prediction.slices[0].physics.location.z]) - car.location)
    avg_speed = 1500
    if car.boost > 50: avg_speed = 2000
    return car_to_ball / avg_speed

def analyze_match_situation(agent):
    ball_loc = agent.ball.location
    my_loc = agent.me.location

    closest_foe = None
    closest_foe_dist = 99999
    for foe in agent.foes:
        dist = np.linalg.norm(foe.location - ball_loc)
        if dist < closest_foe_dist:
            closest_foe_dist = dist
            closest_foe = foe

    my_dist = np.linalg.norm(my_loc - ball_loc)

    if my_dist < closest_foe_dist - 200:
        return 'attacking'
    elif my_dist > closest_foe_dist + 200:
        return 'defending'
    else:
        return 'neutral'

def is_ball_threatening(agent):
    ball_vel = agent.ball.velocity
    ball_loc = agent.ball.location
    goal_loc = agent.friend_goal.location

    to_goal = goal_loc - ball_loc
    angle = angle_between(ball_vel, to_goal)

    if angle < 1.0 and np.linalg.norm(ball_vel) > 500:
        return True
    return False
