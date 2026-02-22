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
        foe_goal = self.agent.foe_goal.location

        # Check kickoff
        if self.agent.kickoff_flag:
            self.agent.push(routines.speed_flip_kickoff())
            return

        # Find possibilities
        targets = {
            "goal": (self.agent.foe_goal.left_post, self.agent.foe_goal.right_post)
        }
        hits = tools.find_hits(self.agent, targets)
        wall_hits = tools.find_wall_hits(self.agent, targets)

        # --- Context Analysis ---
        my_eta, foe_eta = tools.intercept_race(self.agent)
        advantage = my_eta < (foe_eta - 0.1)
        contested = abs(my_eta - foe_eta) < 0.5
        disadvantage = my_eta > foe_eta

        threat_level = self.evaluate_threat()

        # --- Utility Calculation ---
        best_action = None
        highest_score = -1

        # 1. Evaluate Shooting (Standard & Aerial)
        if len(hits["goal"]) > 0:
            for shot in hits["goal"]:
                score = self.score_shot(shot, advantage, contested)
                if score > highest_score:
                    highest_score = score
                    best_action = shot

        # 2. Evaluate Wall Play
        if len(wall_hits) > 0:
             score = 75
             if self.agent.me.boost > 50: score += 10
             if advantage: score += 10

             if score > highest_score:
                 highest_score = score
                 best_action = wall_hits[0]

        # 3. Evaluate Defense / Save
        if threat_level > 80: # High danger
            save_score = 100
            if save_score > highest_score:
                highest_score = save_score
                # Panic save: Go to goal line or intercept
                # Better: Intercept between goal and ball
                target = my_goal + (ball_loc - my_goal) * 0.3
                best_action = routines.goto(target, urgent=True)

        # 4. Evaluate Air Dribble
        dist_to_ball = distance(my_loc, ball_loc)
        if ball_loc[2] > 200 and self.agent.me.boost > 40:
            dribble_score = 65
            if dist_to_ball < 1500: dribble_score += 15
            if advantage: dribble_score += 20 # Only dribble if we have space

            if dribble_score > highest_score:
                highest_score = dribble_score
                best_action = routines.air_dribble()

        # 5. Evaluate Dribble / Pop (Ground)
        if ball_loc[2] < 100 and dist_to_ball < 300 and highest_score < 60:
             dribble_score = 60
             if advantage: dribble_score += 10

             if dribble_score > highest_score:
                 highest_score = dribble_score
                 best_action = routines.air_dribble()

        # 6. Evaluate Fake Challenge (Smart Defense)
        if disadvantage and dist_to_ball < 2500 and threat_level < 80:
             # We are beaten to ball, but close enough to annoy
             fake_score = 70
             if fake_score > highest_score:
                 highest_score = fake_score
                 best_action = routines.fake_challenge()

        # 7. Evaluate Demo Hunt (Aggression)
        # Look for demo if: High boost, not last man back (1v1: always last man, so be careful), or rotating out
        # In 1v1, demo only if ball is safe or on way to ball
        ball_safe = distance(ball_loc, my_goal) > 4000
        if self.agent.me.boost > 30 and (ball_safe or advantage):
            for foe in self.agent.foes:
                if not foe.demolished:
                    d = distance(my_loc, foe.location)
                    # Opportunistic demo
                    if d < 1500 and abs(angle_between(self.agent.me.forward, foe.location - my_loc)) < 0.5:
                        demo_score = 85
                        if demo_score > highest_score:
                            highest_score = demo_score
                            best_action = routines.demo_hunt(foe)

        # 8. Evaluate Boost
        if self.agent.me.boost < 20 and highest_score < 50 and threat_level < 60:
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
                shadow_target = my_goal + np.array([sign(ball_loc[0]) * -800, 0, 0])

            shadow_target[0] = cap(shadow_target[0], -3500, 3500)
            shadow_target[1] = cap(shadow_target[1], -5000, 5000)

            self.agent.push(routines.goto(shadow_target))

    def score_shot(self, shot, advantage, contested):
        time_to_hit = shot.intercept_time - self.agent.time
        if time_to_hit <= 0: return 0

        score = 100
        score -= (time_to_hit * 15)

        if isinstance(shot, routines.aerial_shot) or isinstance(shot, routines.aerial):
            score += 10

        if advantage: score += 20
        if contested: score -= 10

        return cap(score, 0, 100)

    def evaluate_threat(self):
        ball_loc = self.agent.ball.location
        my_goal = self.agent.friend_goal.location

        dist_to_goal = distance(ball_loc, my_goal)

        threat = 0
        if dist_to_goal < 3000: threat += 30
        if dist_to_goal < 1500: threat += 40

        closest_foe_dist = 99999
        for foe in self.agent.foes:
            d = distance(foe.location, ball_loc)
            if d < closest_foe_dist: closest_foe_dist = d

        my_dist = distance(self.agent.me.location, ball_loc)

        if closest_foe_dist < my_dist:
            threat += 30

        return cap(threat, 0, 100)
