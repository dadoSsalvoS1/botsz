from utils import *
from routines import *
from tools import *

class ApexBot(GoslingAgent):
    def run(self):
        # Debug drawing
        self.renderer.draw_string_3d(self.me.location, 2, 2, f"Speed: {round(self.me.velocity.magnitude(), 1)}", self.renderer.white())

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
        # Only do advanced movement mechanics if not in immediate critical danger or trying to shoot
        if situation != 'defending' or not is_ball_threatening(self):
            # If we are on the wall and moving slowly, chain wall dash to gain speed
            if is_wall_dash_viable(self):
                self.push(wall_dash())
                return

            # If we are on the ground and moving slowly, chain wave dash to gain speed
            if is_chain_wave_dash_viable(self):
                self.push(chain_wave_dash())
                return

        # 3. High Level Strategy "The Brain"
        if situation == 'attacking':
            # We have possession or are closest.
            # Tactics: Shoot, Dribble, Air Dribble

            # Define target regions (Goal)
            targets = {
                "1": (self.foe_goal.left_post, self.foe_goal.right_post)
            }

            # Check for air dribble first (Flashy & Effective if space allows)
            if is_air_dribble_viable(self):
                self.push(air_dribble())
                return

            # Try to find a shot (Aerial, Jump, or Ground)
            # determine_shot pushes the routine if found.
            # If defensive=False, it prioritizes speed/power.
            if determine_shot(self, self.foe_goal.location, targets, len(targets), defensive=False):
                return

            # If no shot found, Dribble (Short Shot to goal)
            if len(self.stack) == 0:
                self.push(short_shot(self.foe_goal.location))
                return

        elif situation == 'defending':
            # Enemy has possession or is closer.
            # Tactics: Emergency Save, Shadow Defense, Boost Steal (if safe)

            if is_ball_threatening(self):
                # EMERGENCY: Clear the ball anywhere safe
                # Create wide targets away from our net
                # Using opponent goal as a generic "away" direction for now, or corners
                # Ideally we want to clear to corners, but for now let's just HIT IT.
                targets = {
                    "1": (self.foe_goal.left_post, self.foe_goal.right_post) # Try to clear towards enemy goal
                }
                if determine_shot(self, self.foe_goal.location, targets, len(targets), defensive=True):
                   return

                # If we can't find a smart shot/clear, just drive at the ball to block
                self.push(short_shot(ball_loc))
                return
            else:
                # Shadow Defense
                # Position between ball and goal, but slightly back
                goal_vec = self.friend_goal.location - ball_loc
                target_distance = 1500
                target = ball_loc + goal_vec.normalize() * target_distance

                # Bounds check
                if abs(target.x) > 3500: target.x = 3500 * sign(target.x)

                # If we are low on boost and far from play, maybe grab boost?
                if self.me.boost < 30 and (ball_loc - self.me.location).magnitude() > 2500:
                     self.push(collect_boost())
                     return

                # Urgent return to position
                self.push(goto(target, urgent=True))
                return

        else: # Neutral
            # 50/50 ball or loose ball far away

            # If low boost, prioritise collecting it before engaging
            if self.me.boost < 30 and (ball_loc - self.me.location).magnitude() > 1000:
                self.push(collect_boost())
                return

            # Otherwise, go for the ball (challenge)
            targets = {
                "1": (self.foe_goal.left_post, self.foe_goal.right_post)
            }
            if determine_shot(self, self.foe_goal.location, targets, len(targets), defensive=False):
                return

            # If can't shoot, approach
            self.push(short_shot(ball_loc))
            return
