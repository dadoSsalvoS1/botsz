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

        # --- Context Analysis (Hybrid Brain) ---
        my_eta, foe_eta = tools.intercept_race(self.agent)
        advantage = my_eta < (foe_eta - 0.2) # Clear win
        contested = abs(my_eta - foe_eta) < 0.5
        disadvantage = my_eta > foe_eta

        threat_level = self.evaluate_threat()
        dist_to_ball = distance(my_loc, ball_loc)

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
                target = my_goal + (ball_loc - my_goal) * 0.3
                best_action = routines.goto(target, urgent=True)

        # 4. Evaluate Dribble & Flick (Bumblebee Logic)
        # Ground Control: If ball is low and we have space
        if ball_loc[2] < 120 and dist_to_ball < 200 and advantage:
             dribble_score = 80 # Prioritize possession

             # If carrying but threatened -> Flick
             # Simple heuristic: if we are already dribbling?
             # Brain executes every tick, so we need to detect state or trust `routines` logic.
             # If close to opponent goal or foe is close -> Flick
             foe_dist = min([distance(f.location, my_loc) for f in self.agent.foes])
             if foe_dist < 500 or distance(my_loc, foe_goal) < 1500:
                 dribble_score = 85
                 best_action = routines.flick()
             else:
                 best_action = routines.ground_dribble()

             if dribble_score > highest_score:
                 highest_score = dribble_score

        # 5. Evaluate Air Dribble
        if ball_loc[2] > 200 and self.agent.me.boost > 40 and advantage:
            dribble_score = 65
            if dist_to_ball < 1500: dribble_score += 15

            if dribble_score > highest_score:
                highest_score = dribble_score
                best_action = routines.air_dribble()

        # 6. Evaluate Shadow Defense (Kamael/Cryo Patience)
        # If disadvantaged, don't dive. Shadow.
        if disadvantage and threat_level < 80:
             shadow_score = 90
             # Maintain position between ball and goal, matching lateral movement
             # Calculate shadow target
             ball_to_goal = normalize(my_goal - ball_loc)[0]
             target = ball_loc + ball_to_goal * 1500 # Keep distance
             # Offset to side to cover cutbacks?
             # Simple shadow for now
             if shadow_score > highest_score:
                 highest_score = shadow_score
                 best_action = routines.goto(target, urgent=True)

        # 7. Evaluate Demo Hunt (Aggression)
        ball_safe = distance(ball_loc, my_goal) > 4000
        if self.agent.me.boost > 30 and (ball_safe or advantage):
            for foe in self.agent.foes:
                if not foe.demolished:
                    d = distance(my_loc, foe.location)
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
            # Fallback
            defense_vec, _ = normalize(ball_loc - my_goal)
            shadow_target = ball_loc - defense_vec * 2000

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
