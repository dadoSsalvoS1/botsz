from utils import *
from routines import *
from tools import *

class ApexBot(GoslingAgent):
    def run(self):
        # Debug drawing
        self.renderer.draw_string_3d(self.me.location, 2, 2, f"Speed: {round(self.me.velocity.magnitude(), 1)}", self.renderer.white())

        # If no routine is active, decide on the next one
        if len(self.stack) < 1:
            self.handle_strategy()

    def handle_strategy(self):
        # 1. Kickoff
        if self.kickoff_flag:
            self.push(kickoff())
            return

        # 2. Mechanics & Recovery
        # If we are on the wall and moving slowly, chain wall dash to gain speed
        if is_wall_dash_viable(self):
            self.push(wall_dash())
            return

        # If we are on the ground and moving slowly, chain wave dash to gain speed
        if is_chain_wave_dash_viable(self):
            self.push(chain_wave_dash())
            return

        ball_loc = self.ball.location

        # 3. Role Assignment: Are we the closest teammate to the ball?
        # Include ourselves in the list
        all_cars = self.friends + [self.me]
        # Find car with minimum distance to ball
        closest_car = min(all_cars, key=lambda car: (car.location - ball_loc).magnitude())

        is_closest = (closest_car.index == self.index)

        # 4. Execution
        if is_closest:
            # ATTACK MODE
            # Define target regions (Goal)
            targets = {
                "1": (self.foe_goal.left_post, self.foe_goal.right_post)
            }
            # Try to find a shot (Aerial, Jump, or Ground)
            # determine_shot will push the best shot routine, or a short_shot (dribble) if no shot is found.
            # determine_shot now also checks for air_dribble opportunities
            determine_shot(self, self.foe_goal.location, targets, len(targets))
        else:
            # DEFENSE / SUPPORT MODE (Shadow Defense)

            # If we are low on boost and not in immediate danger, collect boost
            if self.me.boost < 20 and (ball_loc - self.me.location).magnitude() > 2000:
                self.push(collect_boost())
                return

            # Position ourselves between the ball and our goal, acting as a last line of defense
            goal_vec = self.friend_goal.location - ball_loc

            # Target a point 1500 units from the ball towards our goal
            target_distance = 1500
            target = ball_loc + goal_vec.normalize() * target_distance

            # Ensure the target is on our side of the ball relative to the goal (don't go past the ball)
            # Actually, the vector math above ensures we are on the goal side of the ball.

            # Simple bounds checking to stay in field
            if abs(target.x) > 3500: target.x = 3500 * sign(target.x)

            # Go to the defensive position
            self.push(goto(target))
