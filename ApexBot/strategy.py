from utils import *
import routines
import tools
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
            self.agent.push(routines.kickoff())
            return

        # Find possibilities
        targets = {
            "goal": (self.agent.foe_goal.left_post, self.agent.foe_goal.right_post)
        }
        hits = tools.find_hits(self.agent, targets)
        wall_hits = tools.find_wall_hits(self.agent, targets)

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
        if len(wall_hits) > 0:
             # Calculate utility of wall shot
             # If we have a good shot on goal, stick to it, but if wall shot is unexpected/fast, take it.
             # Simple heuristic: if we are close to wall and ball is high
             score = 75
             # Boost bonus
             if self.agent.me.boost > 50: score += 10

             if score > highest_score:
                 highest_score = score
                 best_action = wall_hits[0]

        # 3. Evaluate Defense / Save
        opponent_threat = self.evaluate_threat()
        if opponent_threat > 80: # High danger
            save_score = 100
            if save_score > highest_score:
                highest_score = save_score
                # Panic save: Go to goal line or intercept
                # Better: Intercept between goal and ball
                target = my_goal + (ball_loc - my_goal) * 0.3
                best_action = routines.goto(target, urgent=True)

        # 4. Evaluate Air Dribble
        # Opportunities: Ball high, we have boost, we are close
        dist_to_ball = distance(my_loc, ball_loc)
        if ball_loc[2] > 200 and self.agent.me.boost > 40:
            dribble_score = 65
            if dist_to_ball < 1500: dribble_score += 15

            # Don't dribble if opponent is challenging closely
            if self.evaluate_threat() < 50 and dribble_score > highest_score:
                highest_score = dribble_score
                best_action = routines.air_dribble()

        # 5. Evaluate Dribble / Pop (Ground)
        if ball_loc[2] < 100 and dist_to_ball < 300 and highest_score < 60:
             # We are right next to ball on ground -> start dribble (pop)
             # Reuse air_dribble logic which starts with a pop/lift
             highest_score = 60
             best_action = routines.air_dribble()

        # 6. Evaluate Boost
        if self.agent.me.boost < 20 and highest_score < 50:
            boost_score = 55
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
                best_action = routines.goto_boost(best_boost, ball_loc)

        # Execution
        if best_action:
            self.agent.push(best_action)
        else:
            # Fallback: Shadow Defense
            defense_vec, _ = normalize(ball_loc - my_goal)
            shadow_target = ball_loc - defense_vec * 2000

            # Rotate back post if ball is on side
            if abs(ball_loc[0]) > 2000:
                # Go to far post
                shadow_target = my_goal + np.array([sign(ball_loc[0]) * -800, 0, 0])

            # Clamp
            shadow_target[0] = cap(shadow_target[0], -3500, 3500)
            shadow_target[1] = cap(shadow_target[1], -5000, 5000)

            self.agent.push(routines.goto(shadow_target))

    def score_shot(self, shot):
        time_to_hit = shot.intercept_time - self.agent.time
        if time_to_hit <= 0: return 0

        # Base score starts high
        score = 100

        # Penalize slowness
        score -= (time_to_hit * 15)

        # Bonus for aerials (harder to save)
        if isinstance(shot, routines.aerial_shot) or isinstance(shot, routines.aerial):
            score += 10

        return cap(score, 0, 100)

    def evaluate_threat(self):
        ball_loc = self.agent.ball.location
        my_goal = self.agent.friend_goal.location

        dist_to_goal = distance(ball_loc, my_goal)

        threat = 0
        if dist_to_goal < 3000: threat += 30
        if dist_to_goal < 1500: threat += 40

        # Check if opponent is closer to ball than us
        closest_foe_dist = 99999
        for foe in self.agent.foes:
            d = distance(foe.location, ball_loc)
            if d < closest_foe_dist: closest_foe_dist = d

        my_dist = distance(self.agent.me.location, ball_loc)

        if closest_foe_dist < my_dist:
            threat += 30

        return cap(threat, 0, 100)
