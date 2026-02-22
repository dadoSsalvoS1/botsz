from routines import *
from utils import *

#This file is for strategic tools

def find_hits(agent,targets):
    #find_hits takes a dict of (left,right) target pairs and finds routines that could hit the ball between those target pairs
    #find_hits is only meant for routines that require a defined intercept time/place in the future
    #find_hits should not be called more than once in a given tick, as it has the potential to use an entire tick to calculate

    #Example Useage:
    #targets = {"goal":(opponent_left_post,opponent_right_post), "anywhere_but_my_net":(my_right_post,my_left_post)}
    #hits = find_hits(agent,targets)
    #print(hits)
    #>{"goal":[a ton of jump and aerial routines,in order from soonest to latest], "anywhere_but_my_net":[more routines and stuff]}
    hits = {name:[] for name in targets}
    struct = agent.get_ball_prediction_struct()

    #Begin looking at slices 0.25s into the future
    #The number of slices
    i = 15
    while i < struct.num_slices:
        #Gather some data about the slice
        intercept_time = struct.slices[i].game_seconds
        time_remaining = intercept_time - agent.time
        if time_remaining > 0:
            ball_location = Vector3(struct.slices[i].physics.location)
            ball_velocity = Vector3(struct.slices[i].physics.velocity).magnitude()

            if abs(ball_location[1]) > 5250:
                break #abandon search if ball is scored at/after this point

            #determine the next slice we will look at, based on ball velocity (slower ball needs fewer slices)
            i += 15 - cap(int(ball_velocity//150),0,13)

            car_to_ball = ball_location - agent.me.location
            #Adding a True to a vector's normalize will have it also return the magnitude of the vector
            direction, distance = car_to_ball.normalize(True)

            #How far the car must turn in order to face the ball, for forward and reverse
            forward_angle = direction.angle(agent.me.forward)
            backward_angle = math.pi - forward_angle

            #Accounting for the average time it takes to turn and face the ball
            #Backward is slightly longer as typically the car is moving forward and takes time to slow down
            forward_time = time_remaining - (forward_angle * 0.318)
            backward_time = time_remaining - (backward_angle * 0.418)

            #If the car only had to drive in a straight line, we ensure it has enough time to reach the ball (a few assumptions are made)
            forward_flag = forward_time > 0.0 and (distance*1.05 / forward_time) < (2290 if agent.me.boost > distance/100 else 1400)
            backward_flag = distance < 1500 and backward_time > 0.0 and (distance*1.05 / backward_time) < 1200

            #Provided everything checks out, we begin to look at the target pairs
            if forward_flag or backward_flag:
                for pair in targets:
                    #First we correct the target coordinates to account for the ball's radius
                    #If swapped == True, the shot isn't possible because the ball wouldn't fit between the targets
                    left,right,swapped = post_correction(ball_location,targets[pair][0],targets[pair][1])
                    if not swapped:
                        #Now we find the easiest direction to hit the ball in order to land it between the target points
                        left_vector = (left - ball_location).normalize()
                        right_vector = (right - ball_location).normalize()
                        best_shot_vector = direction.clamp(left_vector,right_vector)

                        #Check to make sure our approach is inside the field
                        if in_field(ball_location - (200*best_shot_vector),1):
                            #The slope represents how close the car is to the chosen vector, higher = better
                            #A slope of 1.0 would mean the car is 45 degrees off
                            slope = find_slope(best_shot_vector,car_to_ball)
                            if forward_flag:
                                if ball_location[2] <= 300 and slope > 0.0:
                                    hits[pair].append(jump_shot(ball_location,intercept_time,best_shot_vector,slope))
                                if ball_location[2] > 300 and ball_location[2] < 600 and slope > 1.0 and (ball_location[2]-250) * 0.14 > agent.me.boost:
                                    hits[pair].append(aerial_shot(ball_location,intercept_time,best_shot_vector,slope))
                                if ball_location.z > 600:
                                    shot = aerial(ball_location - 92 * best_shot_vector, intercept_time, True,
                                                    target=best_shot_vector)
                                    if shot.is_viable(agent, agent.time) and should_aerial(agent, shot):
                                        hits[pair].append(shot)
                            elif backward_flag and ball_location[2] <= 280 and slope > 0.25:
                                hits[pair].append(jump_shot(ball_location,intercept_time,best_shot_vector,slope,-1))
        else:
            i += 1
    return hits


def determine_shot(agent, target, targets, target_count, defensive=False, center=False):
    if agent.ball.velocity.magnitude() > 0:
        hits = find_hits(agent, targets)
        if len(hits):
            pick_the_fastest = []
            for i in range(1, 1 + target_count):
                if len(hits[str(i)]):
                    shot = hits[str(i)][0]
                    # dont go for hits were we have to crawl to
                    hit_location = shot.ball_location
                    hit_time = shot.intercept_time
                    time_delta = hit_time - agent.time
                    location_delta = (agent.me.location - hit_location).magnitude()
                    avg_speed = location_delta / time_delta
                    # print(agent.index, location_delta, time_delta, avg_speed, agent.me.boost)
                    # sorted_foes = agent.foes[:]
                    # sorted_foes.sort(key=lambda car: (hit_location - car.location).magnitude())
                    # closest_enemy_distance = (hit_location - sorted_foes[0].location).magnitude()
                    pick_the_fastest.append(shot)
                    if (avg_speed < 700):
                        continue
                    # max_boost_needed
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
                return defensive
    if center: return False
    if len(agent.stack): agent.pop()
    shot = short_shot(target)
    agent.push(shot)
    return not center


def determine_follow_up_shot(agent, targets, target_count):
    if agent.ball.velocity.magnitude() > 0:
        hits = find_hits(agent, targets)
        if len(hits):
            for i in range(1, 1 + target_count):
                if len(hits[str(i)]):
                    for shot in hits[str(i)]:
                        if type(shot) != aerial:
                            continue
                        else:
                            agent.aerialing = False
                            if len(agent.stack): agent.pop()
                            agent.push(shot)
                            return True
    return False


def should_aerial(agent, shot:aerial):
    # Check if we have enough boost
    return agent.me.boost > 30

def is_air_dribble_viable(agent):
    # Ball should be high, near us, and we should have boost
    dist = (agent.ball.location - agent.me.location).magnitude()
    if agent.ball.location.z > 200 and dist < 1000 and agent.me.boost > 50:
        return True
    return False

def is_wall_dash_viable(agent):
    # On wall and needs speed
    if agent.me.airborne: return False
    if abs(agent.me.up.z) > 0.7: return False # On floor or ceiling
    if agent.me.velocity.magnitude() > 2200: return False # Already fast
    return True

def is_chain_wave_dash_viable(agent):
    # On ground, moving slowly, upright
    if agent.me.airborne: return False
    if agent.me.velocity.magnitude() > 1500: return False
    if agent.me.up.z < 0.8: return False # Not upright enough
    return True

# --- NEW STRATEGIC ANALYSIS FUNCTIONS ---

def intercept_time(car, ball_prediction):
    # Estimator for time to reach ball.
    # Uses a simple physics model: distance / average_speed
    # This is a heuristic and not as accurate as `find_hits` but faster for general logic.
    car_to_ball = (Vector3(ball_prediction.slices[0].physics.location) - car.location).magnitude()
    avg_speed = 1500 # Assume average game speed
    if car.boost > 50: avg_speed = 2000

    return car_to_ball / avg_speed

def analyze_match_situation(agent):
    # High-level state classification

    ball_loc = agent.ball.location
    my_loc = agent.me.location

    # Check possession/pressure
    # Are we closer than the closest enemy?
    closest_foe = None
    closest_foe_dist = 99999
    for foe in agent.foes:
        dist = (foe.location - ball_loc).magnitude()
        if dist < closest_foe_dist:
            closest_foe_dist = dist
            closest_foe = foe

    my_dist = (my_loc - ball_loc).magnitude()

    if my_dist < closest_foe_dist - 200:
        return 'attacking'
    elif my_dist > closest_foe_dist + 200:
        return 'defending'
    else:
        return 'neutral' # 50/50 territory

def is_ball_threatening(agent):
    # Is the ball moving towards our goal?
    ball_vel = agent.ball.velocity
    ball_loc = agent.ball.location
    goal_loc = agent.friend_goal.location

    # Vector from ball to goal
    to_goal = goal_loc - ball_loc

    # Angle between velocity and direction to goal
    angle = ball_vel.angle(to_goal)

    # If angle is small (moving towards goal) and speed is decent
    if angle < 1.0 and ball_vel.magnitude() > 500:
        return True
    return False
