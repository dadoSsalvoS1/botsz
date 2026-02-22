from utils import *
from routines import *
from tools import *

class Brain:
    def __init__(self):
        pass

    def run(self, agent):
        # 1. Kickoff
        if agent.kickoff_flag:
            if len(agent.stack) < 1:
                agent.push(kickoff())
            return

        # 2. Information Gathering
        ball_loc = agent.ball.location
        my_loc = agent.me.location

        # Determine if we are closest
        all_cars = agent.friends + [agent.me]
        closest_car = min(all_cars, key=lambda car: (car.location - ball_loc).magnitude())
        is_closest = (closest_car.index == agent.index)

        # Intercept calculation
        my_time, foe_time = intercept_race(agent)

        # 3. Decision Logic
        if len(agent.stack) < 1:
            if is_closest:
                # ATTACK

                # Check for shot
                targets = {
                    "goal": (agent.foe_goal.left_post, agent.foe_goal.right_post)
                }

                # Try to find a shot
                if determine_shot(agent, agent.foe_goal.location, targets, len(targets)):
                    return

                # If no shot found, consider dribble if close
                dist_to_ball = (ball_loc - my_loc).magnitude()
                if dist_to_ball < 200 and ball_loc.z < 100:
                    # Check if 'carry' exists in routines (it will be added)
                    if 'carry' in globals():
                        agent.push(carry())
                        return

                # Default attack: drive to ball
                agent.push(short_shot(agent.foe_goal.location))

            else:
                # DEFENSE / SUPPORT

                # If opponent is closer and ball is threatening, Shadow Defense
                if foe_time < my_time + 0.5:
                     if 'shadow_defense' in globals():
                         agent.push(shadow_defense())
                         return

                # Otherwise, go to defensive position
                goal_vec = agent.friend_goal.location - ball_loc
                target = ball_loc + goal_vec.normalize() * 1500

                # Bounds check
                target.x = cap(target.x, -3500, 3500)
                target.y = cap(target.y, -5000, 5000)

                agent.push(goto(target))
