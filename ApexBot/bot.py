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
        # REMOVED GLOBAL WALL DASH AND WAVE DASH SPAM
        # Mechanics are now integrated into specific routines like goto() or strategy blocks

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
                # Shadow Defense / Backpost Rotation

                # If a teammate is closer than us, we should rotate to backpost
                # Find teammate closest to ball
                closest_teammate = None
                closest_dist = 99999
                for car in self.friends:
                    d = (car.location - ball_loc).magnitude()
                    if d < closest_dist:
                        closest_dist = d
                        closest_teammate = car

                am_i_closest = True
                if closest_teammate and closest_dist < (self.me.location - ball_loc).magnitude():
                    am_i_closest = False

                if not am_i_closest:
                    # Rotate to Backpost
                    # Backpost is the goal post furthest from the ball
                    left_post_dist = (self.friend_goal.left_post - ball_loc).magnitude()
                    right_post_dist = (self.friend_goal.right_post - ball_loc).magnitude()

                    if left_post_dist > right_post_dist:
                        target = self.friend_goal.left_post
                    else:
                        target = self.friend_goal.right_post

                    # Move *inside* the goal slightly to face out
                    # This is simple: just target the post for now, maybe offset slightly
                    # Using goto with vector pointing OUT of goal (towards center field)
                    center_field = Vector3(0, 0, 0)
                    self.push(goto(target, vector=center_field - target))
                    return
                else:
                    # We are the last line of defense (Shadow)
                    # Position between ball and goal, but slightly back
                    goal_vec = self.friend_goal.location - ball_loc
                    target_distance = 1500
                    target = ball_loc + goal_vec.normalize() * target_distance

                    # Bounds check
                    if abs(target.x) > 3500: target.x = 3500 * sign(target.x)

                    # If we are low on boost and far from play, maybe grab boost?
                    # Only if ball is far away (> 3000) to avoid leaving net open
                    if self.me.boost < 30 and (ball_loc - self.me.location).magnitude() > 3000:
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
