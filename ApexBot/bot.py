from utils import *
from routines import *
from tools import *
import numpy as np

class ApexBot(GoslingAgent):
    def run(self):
        # Debug drawing
        self.renderer.draw_string_3d(self.me.location, 2, 2, f"Speed: {round(np.linalg.norm(self.me.velocity), 1)}", self.renderer.white())

        # State Debug
        situation = analyze_match_situation(self)
        self.renderer.draw_string_2d(20, 200, 2, 2, f"State: {situation}", self.renderer.yellow())

        # If no routine is active, decide on the next one
        if len(self.stack) < 1:
            self.handle_strategy()

    def handle_strategy(self):
        # 1. Kickoff
        if self.kickoff_flag:
            self.push(kickoff())
            return

        ball_loc = self.ball.location
        situation = analyze_match_situation(self)

        # 2. Mechanics & Recovery
        # If we are on the wall and moving slowly, chain wall dash to gain speed
        if is_wall_dash_viable(self):
            # No longer need separate check, wall_dash handles its own logic but we can push it if needed
            # Actually, let's keep it clean: movement routines are generally called inside other tasks or explicitly
            # But the user asked for it to be smart.
            # If we are not doing anything else important and we are on the wall, go fast.
            # But we are in handle_strategy, meaning we HAVE NO ROUTINE.
            # So yes, we can push a wall dash if we are just traversing.
            # But usually we want to go SOMEWHERE.
            # Let's rely on goto() integrating it or push it if just recovery.
            pass

        # 3. High Level Strategy "The Brain"
        if situation == 'attacking':
            # Define target regions (Goal)
            targets = {
                "1": (self.foe_goal.left_post, self.foe_goal.right_post)
            }

            # Check for air dribble
            if is_air_dribble_viable(self):
                self.push(air_dribble())
                return

            # Try to find a shot
            if determine_shot(self, self.foe_goal.location, targets, len(targets), defensive=False):
                return

            # If no shot found, Dribble
            if len(self.stack) == 0:
                self.push(short_shot(self.foe_goal.location))
                return

        elif situation == 'defending':
            if is_ball_threatening(self):
                targets = {
                    "1": (self.foe_goal.left_post, self.foe_goal.right_post)
                }
                if determine_shot(self, self.foe_goal.location, targets, len(targets), defensive=True):
                   return

                self.push(short_shot(ball_loc))
                return
            else:
                # Shadow Defense / Backpost Rotation
                closest_teammate = None
                closest_dist = 99999
                for car in self.friends:
                    d = np.linalg.norm(car.location - ball_loc)
                    if d < closest_dist:
                        closest_dist = d
                        closest_teammate = car

                am_i_closest = True
                if closest_teammate and closest_dist < np.linalg.norm(self.me.location - ball_loc):
                    am_i_closest = False

                if not am_i_closest:
                    # Rotate to Backpost
                    left_post_dist = np.linalg.norm(self.friend_goal.left_post - ball_loc)
                    right_post_dist = np.linalg.norm(self.friend_goal.right_post - ball_loc)

                    if left_post_dist > right_post_dist:
                        target = self.friend_goal.left_post
                    else:
                        target = self.friend_goal.right_post

                    center_field = np.array([0, 0, 0])
                    self.push(goto(target, vector=center_field - target))
                    return
                else:
                    # Shadow
                    goal_vec = self.friend_goal.location - ball_loc
                    target_distance = 1500
                    target = ball_loc + normalize(goal_vec) * target_distance

                    if abs(target[0]) > 3500: target[0] = 3500 * sign(target[0])

                    if self.me.boost < 30 and np.linalg.norm(ball_loc - self.me.location) > 3000:
                         self.push(collect_boost())
                         return

                    self.push(goto(target, urgent=True))
                    return

        else: # Neutral
            if self.me.boost < 30 and np.linalg.norm(ball_loc - self.me.location) > 1000:
                self.push(collect_boost())
                return

            targets = {
                "1": (self.foe_goal.left_post, self.foe_goal.right_post)
            }
            if determine_shot(self, self.foe_goal.location, targets, len(targets), defensive=False):
                return

            self.push(short_shot(ball_loc))
            return
