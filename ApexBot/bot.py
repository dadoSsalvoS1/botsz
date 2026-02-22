from utils import *
from routines import *
from tools import *
import numpy as np
import traceback

class ApexBot(GoslingAgent):
    def run(self):
        try:
            # Visualization
            speed = np.linalg.norm(self.me.velocity)
            self.renderer.draw_string_3d(self.me.location, 2, 2, f"Speed: {round(speed, 1)}", self.renderer.white())
            self.renderer.draw_string_3d(self.me.location + np.array([0,0,50]), 2, 2, f"Stack: {len(self.stack)}", self.renderer.white())

            if len(self.stack) < 1:
                self.handle_strategy()
        except Exception:
            self.renderer.draw_string_2d(10, 10, 3, 3, f"CRASH: {traceback.format_exc().splitlines()[-1]}", self.renderer.red())
            print(traceback.format_exc())
            if len(self.stack) == 0:
                self.push(recovery()) # Try to recover

    def handle_strategy(self):
        # 1. Kickoff
        if self.kickoff_flag:
            self.push(speed_flip(self.ball.location))
            self.kickoff_flag = False
            return

        # 2. Team Structure
        # Find closest teammate to ball
        all_cars = self.friends + [self.me]
        closest_car = min(all_cars, key=lambda car: np.linalg.norm(car.location - self.ball.location))
        is_closest = (closest_car.index == self.index)

        # 3. Decision Logic
        if is_closest:
            # ATTACK
            targets = {
                "1": (self.foe_goal.left_post, self.foe_goal.right_post),
                "2": (self.foe_goal.left_post, self.foe_goal.right_post)
            }

            # Search for shots
            found_shot = determine_shot(self, self.foe_goal.location, targets, len(targets))

            if not found_shot and len(self.stack) == 0:
                # Fallback: Drive to ball (short_shot handles basic approach)
                # determine_shot pushes short_shot if generic driving is needed, but just in case
                self.push(short_shot(self.foe_goal.location))

        else:
            # DEFENSE / SUPPORT
            ball_loc = self.ball.location
            goal_loc = self.friend_goal.location

            # Vector from goal to ball
            goal_to_ball = ball_loc - goal_loc
            dist_to_ball = np.linalg.norm(goal_to_ball)

            # Target position: Shadow Defense
            target_dist = 2000 # Default shadow distance
            if dist_to_ball < 2000: target_dist = dist_to_ball * 0.5

            # Calculate shadow target
            # Normalize vector
            if dist_to_ball > 0:
                dir_to_ball = goal_to_ball / dist_to_ball
                shadow_target = ball_loc - dir_to_ball * target_dist
            else:
                shadow_target = goal_loc

            if in_goal_area(self):
                 shadow_target = goal_loc + (ball_loc - goal_loc) * 0.1

            # If we have low boost, look for boost
            if self.me.boost < 30:
                closest_boost = None
                closest_dist = 99999
                for boost in self.boosts:
                    if boost.active and boost.large:
                        dist = np.linalg.norm(boost.location - self.me.location)
                        if dist < closest_dist:
                            closest_dist = dist
                            closest_boost = boost

                if closest_boost and closest_dist < 3000:
                    self.push(goto_boost(closest_boost, self.ball.location))
                    return

            self.push(goto(shadow_target, self.ball.location))
