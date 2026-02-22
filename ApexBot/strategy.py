from utils import *
from routines import *
from tools import *
import numpy as np

class Brain:
    def __init__(self, agent):
        self.agent = agent
        self.state = "idle" # idle, attacking, defending, gathering_boost, clearing
        self.last_action_time = 0

    def execute(self):
        # 1. State Analysis
        ball_loc = self.agent.ball.location
        my_loc = self.agent.me.location
        my_goal = self.agent.friend_goal.location
        foe_goal = self.agent.foe_goal.location

        dist_to_ball = distance(my_loc, ball_loc)

        # Check kickoff
        if self.agent.kickoff_flag:
            self.agent.push(kickoff())
            return

        # Find shots
        targets = {
            "goal": (self.agent.foe_goal.left_post, self.agent.foe_goal.right_post)
        }
        hits = find_hits(self.agent, targets)

        # Check boost
        need_boost = self.agent.me.boost < 30

        # Calculate intercept times
        my_intercept_time = 999
        best_shot = None

        if len(hits["goal"]) > 0:
            best_shot = hits["goal"][0] # Hits are sorted by time? No, find_hits appends in order of slices (time)
            my_intercept_time = best_shot.intercept_time - self.agent.time

        # Opponent intercept prediction (simplified)
        opponent_intercept_time = 999
        closest_foe = None
        closest_foe_dist = 99999

        if len(self.agent.foes) > 0:
            for foe in self.agent.foes:
                d = distance(foe.location, ball_loc)
                if d < closest_foe_dist:
                    closest_foe_dist = d
                    closest_foe = foe

            # Rough estimate: distance / average speed (e.g. 1500)
            opponent_intercept_time = closest_foe_dist / 1500

        # Decision Matrix

        # 1. Clear Shot on Goal (High Priority)
        if best_shot and my_intercept_time < opponent_intercept_time - 0.5:
            # We can beat them to the ball comfortably
            self.agent.push(best_shot)
            return

        # 2. Defensive Clear / Save
        ball_in_our_half = dot(ball_loc - my_goal, self.agent.friend_goal.location - self.agent.foe_goal.location) > 0 # Wait, dot product direction?
        # Friend goal at Y= +/- 5100. Vector from center to friend goal.
        # Simple check:
        # If team 0 (Blue, Y=-5120), balls with Y < 0 are in our half.
        # If team 1 (Orange, Y=5120), balls with Y > 0 are in our half.

        in_our_half = False
        if self.agent.team == 0:
            if ball_loc[1] < 0: in_our_half = True
        else:
            if ball_loc[1] > 0: in_our_half = True

        if in_our_half and opponent_intercept_time < 3.0:
            # Panic defense
            # Try to hit it anywhere away from our net
            # For now, just simplistic defense
            defense_target = self.agent.friend_goal.location + (ball_loc - self.agent.friend_goal.location) * 0.5
            self.agent.push(goto(defense_target, urgent=True))
            return

        # 3. Air Dribble Opportunity (Ball bouncing high or near wall)
        if ball_loc[2] > 200 and self.agent.me.boost > 50:
             # Basic heuristic for air dribble
             # If we are close and ball is going up
             if dist_to_ball < 1000 and self.agent.ball.velocity[2] > 100:
                 self.agent.push(air_dribble())
                 return

        # 4. Gather Boost
        if need_boost:
            # Find closest boost
            best_boost = None
            best_dist = 99999
            for boost in self.agent.boosts:
                if boost.active and boost.large:
                    d = distance(my_loc, boost.location)
                    if d < best_dist:
                        best_dist = d
                        best_boost = boost

            if best_boost and best_dist < 3000: # Only go if reasonably close
                self.agent.push(goto_boost(best_boost, self.agent.ball.location))
                return

        # 5. Default / Positioning (Shadow Defense)
        # Stay between ball and goal
        defense_vec = (ball_loc - my_goal)
        defense_vec, _ = normalize(defense_vec)
        shadow_target = ball_loc - defense_vec * 1500

        # Clamp target to field
        shadow_target[0] = cap(shadow_target[0], -3500, 3500)
        shadow_target[1] = cap(shadow_target[1], -5000, 5000)

        self.agent.push(goto(shadow_target))
