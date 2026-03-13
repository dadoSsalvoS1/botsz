import numpy as np
import utils
import routines
import tools

class Brain:
    """
    Cérebro estratégico do ApexBot.
    Decide qual rotina executar baseada no estado do jogo, posicionamento e ameaças.
    Implementa uma estratégia híbrida: agressiva, defensiva e de vantagem.
    """
    def __init__(self, agent):
        self.agent = agent

    def execute(self):
        # Se já houver uma rotina ativa na pilha, deixamos ela rodar (exceto se for interrompível)
        if len(self.agent.stack) > 0:
            return

        # 1. Kickoff
        if self.agent.kickoff_flag:
            self.agent.push(routines.kickoff())
            return

        # 2. Defesa de Emergência / Recuperação
        if self.agent.me.airborne and not self.agent.me.supersonic:
            # Se estivermos no ar sem controle, tentamos recuperar
            self.agent.push(routines.recovery())
            return

        # 3. Análise de Posicionamento e Alvos
        ball_loc = self.agent.ball.location
        me_loc = self.agent.me.location

        # Encontra o carro mais próximo da bola (aliado ou o próprio agente)
        all_friends = self.agent.friends + [self.agent.me]
        closest_friend = min(all_friends, key=lambda car: utils.magnitude(car.location - ball_loc))
        is_closest = (closest_friend.index == self.agent.index)

        # 4. Tomada de Decisão Baseada em Papel
        if is_closest:
            # MODO ATAQUE: Somos os principais responsáveis pela bola
            targets = {
                "1": (self.agent.foe_goal.left_post, self.agent.foe_goal.right_post)
            }

            # Tenta encontrar um chute otimizado
            # determine_shot decidirá entre jump_shot, aerial ou short_shot
            tools.determine_shot(self.agent, self.agent.foe_goal.location, targets, len(targets))
        else:
            # MODO SUPORTE / SHADOW DEFENSE
            # Posicionamento defensivo entre a bola e o nosso gol
            goal_to_ball = ball_loc - self.agent.friend_goal.location
            goal_to_ball_dist = utils.magnitude(goal_to_ball)

            # Queremos ficar a uma distância segura da bola, recuando
            target_distance = 2000
            if goal_to_ball_dist > 0:
                defensive_pos = self.agent.friend_goal.location + (goal_to_ball / goal_to_ball_dist) * (goal_to_ball_dist - target_distance)
            else:
                defensive_pos = self.agent.friend_goal.location

            # Limites do campo para evitar bater nas paredes
            defensive_pos[0] = utils.cap(defensive_pos[0], -3000, 3000)
            defensive_pos[1] = utils.cap(defensive_pos[1], -5000, 5000)

            # Se estivermos muito longe da posição ideal, vamos até lá
            if utils.magnitude(me_loc - defensive_pos) > 500:
                self.agent.push(routines.goto(defensive_pos, urgent=(utils.magnitude(self.agent.ball.velocity) > 1000)))
            else:
                # Caso contrário, apenas encaramos a bola
                local_ball = self.agent.me.local(ball_loc - me_loc)
                utils.defaultPD(self.agent, local_ball)
                utils.defaultThrottle(self.agent, 0) # Fica parado ou ajusta levemente

        # 5. Gestão de Boost (Oportunista)
        if self.agent.me.boost < 30 and len(self.agent.stack) == 0:
            # Procura o boost grande mais próximo que esteja ativo
            active_boosts = [b for b in self.agent.boosts if b.active and b.large]
            if active_boosts:
                closest_boost = min(active_boosts, key=lambda b: utils.magnitude(b.location - me_loc))
                # Só vamos buscar se não estivermos muito longe da defesa
                if utils.magnitude(closest_boost.location - ball_loc) < 4000:
                     self.agent.push(routines.goto_boost(closest_boost, ball_loc))
