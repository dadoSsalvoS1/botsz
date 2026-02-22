from utils import *
from routines import *
from tools import *
import numpy as np

class Brain:
    def __init__(self, agent):
        self.agent = agent
        self.last_action_time = 0

    def execute(self):
        ball_loc = self.agent.ball.location
        my_loc = self.agent.me.location
        my_goal = self.agent.friend_goal.location

        # Check kickoff
        if self.agent.kickoff_flag:
            self.agent.push(kickoff())
            return

        # Find possibilities
        targets = {
            "goal": (self.agent.foe_goal.left_post, self.agent.foe_goal.right_post)
        }
        hits = find_hits(self.agent, targets)
        wall_hits = find_wall_hits(self.agent, targets)

        # --- Utility Calculation ---
        best_action = None
        highest_score = -1

        # 1. Evaluate Shooting (Standard & Aerial)
        if len(hits["goal"]) > 0:
            for shot in hits["goal"]:
                score = self.score_shot(shot)
                if score > highest_score:
                    highest_score = score
                    best_action = shot

        # 2. Evaluate Wall Play
        if len(wall_hits) > 0 and highest_score < 80: # Only if no guaranteed goal
             score = 75 # Static high score for now if valid
             if score > highest_score:
                 highest_score = score
                 best_action = wall_hits[0]

        # 3. Evaluate Defense / Save
        opponent_threat = self.evaluate_threat()
        if opponent_threat > 80:
            # Panic save
            save_score = 100 # Override everything
            if save_score > highest_score:
                highest_score = save_score
                # Construct defensive move (e.g., intercept or shadow)
                target = my_goal + (ball_loc - my_goal) * 0.5
                best_action = goto(target, urgent=True)

        # 4. Evaluate Air Dribble
        if ball_loc[2] > 150 and self.agent.me.boost > 40 and highest_score < 70:
            dribble_score = 70
            if dribble_score > highest_score:
                highest_score = dribble_score
                best_action = air_dribble()

        # 5. Evaluate Boost
        if self.agent.me.boost < 20 and highest_score < 50:
            boost_score = 60
            # Find best boost
            best_boost = None
            min_dist = 9999
            for b in self.agent.boosts:
                if b.active and b.large:
                    d = distance(my_loc, b.location)
                    if d < min_dist:
                        min_dist = d
                        best_boost = b

            if best_boost:
                highest_score = boost_score
                best_action = goto_boost(best_boost, ball_loc)

        # Execution
        if best_action:
            self.agent.push(best_action)
        else:
            # Fallback: Shadow Defense
            defense_vec, _ = normalize(ball_loc - my_goal)
            shadow_target = ball_loc - defense_vec * 2000
            # Clamp
            shadow_target[0] = cap(shadow_target[0], -3000, 3000)
            shadow_target[1] = cap(shadow_target[1], -4500, 4500)
            self.agent.push(goto(shadow_target))

    def score_shot(self, shot):
        # Score based on speed and intercept time
        # Faster intercept = higher score
        time_to_hit = shot.intercept_time - self.agent.time
        if time_to_hit <= 0: return 0

        score = 100 - (time_to_hit * 10) # Decay score over time

        # Bonus for goal shot (which it is, since it came from 'goal' target)
        score += 20

        return cap(score, 0, 100)

    def evaluate_threat(self):
        # Simple threat: ball near our goal and opponent closer than us
        ball_loc = self.agent.ball.location
        my_goal = self.agent.friend_goal.location
        dist_to_goal = distance(ball_loc, my_goal)

        if dist_to_goal < 2000:
            return 90
        return 0
