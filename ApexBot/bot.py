from utils import *
from routines import *
from tools import *

class ApexBot(GoslingAgent):
    def run(self):
        # Debug drawing
        # loc = game_data_struct.Vector3(self.me.location[0], self.me.location[1], self.me.location[2])
        # We use SimpleVector3 for rendering now
        loc = SimpleVector3(self.me.location[0], self.me.location[1], self.me.location[2])
        self.renderer.draw_string_3d(loc, 2, 2, f"Speed: {round(magnitude(self.me.velocity), 1)}", self.renderer.white())

        # If no routine is active, decide on the next one
        if len(self.stack) < 1:
            self.handle_strategy()

    def handle_strategy(self):
        # 1. Kickoff
        if self.kickoff_flag:
            self.push(kickoff())
            return

        ball_loc = self.ball.location

        # 2. Role Assignment: Are we the closest teammate to the ball?
        # Include ourselves in the list
        all_cars = self.friends + [self.me]
        # Find car with minimum distance to ball
        closest_car = min(all_cars, key=lambda car: magnitude(car.location - ball_loc))

        is_closest = (closest_car.index == self.index)

        # 3. Execution
        if is_closest:
            # ATTACK MODE
            # Define target regions (Goal)
            targets = {
                "1": (self.foe_goal.left_post, self.foe_goal.right_post)
            }
            # Try to find a shot (Aerial, Jump, or Ground)
            # determine_shot will push the best shot routine, or a short_shot (dribble) if no shot is found.
            determine_shot(self, self.foe_goal.location, targets, len(targets))
        else:
            # DEFENSE / SUPPORT MODE (Shadow Defense)
            # Position ourselves between the ball and our goal, acting as a last line of defense
            goal_vec = self.friend_goal.location - ball_loc

            # Target a point 1500 units from the ball towards our goal
            target_distance = 1500

            # target = ball_loc + goal_vec.normalize() * target_distance
            gv_norm, _ = normalize(goal_vec)
            target = ball_loc + gv_norm * target_distance

            # Ensure the target is on our side of the ball relative to the goal (don't go past the ball)
            # Actually, the vector math above ensures we are on the goal side of the ball.

            # Simple bounds checking to stay in field
            # if abs(target.x) > 3500: target.x = 3500 * sign(target.x)
            if abs(target[0]) > 3500: target[0] = 3500 * sign(target[0])

            # Go to the defensive position
            self.push(goto(target))
