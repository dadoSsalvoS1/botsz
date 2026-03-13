import math
import numpy as np
import routines
import utils

# Este arquivo contém ferramentas estratégicas para análise de jogadas e previsões.

def find_hits(agent, targets):
    """
    Analisa a trajetória da bola e identifica possíveis rotinas (chutes) que podem
    levar a bola até os alvos definidos.
    Refatorado para usar NumPy e maior resolução de amostragem.
    """
    hits = {name: [] for name in targets}
    struct = agent.get_ball_prediction_struct()

    # Começamos analisando a partir de 0.1s no futuro para maior precisão
    i = 5
    while i < struct.num_slices:
        slice = struct.slices[i]
        intercept_time = slice.game_seconds
        time_remaining = intercept_time - agent.time

        if time_remaining <= 0:
            i += 1
            continue

        ball_location = np.array([slice.physics.location.x, slice.physics.location.y, slice.physics.location.z])
        ball_velocity_vec = np.array([slice.physics.velocity.x, slice.physics.velocity.y, slice.physics.velocity.z])
        ball_speed = utils.magnitude(ball_velocity_vec)

        # Se a bola entrar no gol, interrompemos a busca nesta trajetória
        if abs(ball_location[1]) > 5250:
            break

        # Salto dinâmico de fatias baseado na velocidade da bola
        i += max(1, 15 - utils.cap(int(ball_speed // 150), 0, 13))

        car_to_ball = ball_location - agent.me.location
        distance = utils.magnitude(car_to_ball)
        direction = utils.normalize(car_to_ball)

        # Cálculo do tempo de giro (aproximação)
        forward_angle = utils.angle_between(direction, agent.me.forward)
        backward_angle = math.pi - forward_angle

        forward_time = time_remaining - (forward_angle * 0.318)
        backward_time = time_remaining - (backward_angle * 0.418)

        # Verificação se o agente consegue chegar na bola a tempo
        forward_flag = forward_time > 0.0 and (distance * 1.05 / forward_time) < (2290 if agent.me.boost > distance/100 else 1400)
        backward_flag = distance < 1500 and backward_time > 0.0 and (distance * 1.05 / backward_time) < 1200

        if forward_flag or backward_flag:
            for pair in targets:
                left_post = targets[pair][0]
                right_post = targets[pair][1]

                # Ajuste de poste considerando o raio da bola
                left, right, swapped = utils.post_correction(ball_location, left_post, right_post)

                if not swapped:
                    # Vetor de chute ideal entre os postes
                    left_vector = utils.normalize(left - ball_location)
                    right_vector = utils.normalize(right - ball_location)

                    # Clamp do vetor de direção atual entre os limites do gol
                    target_center = (left + right) / 2
                    best_shot_vector = utils.normalize(target_center - ball_location)

                    # Verifica se a posição de abordagem está dentro do campo
                    if utils.in_field(ball_location - (200 * best_shot_vector), 1):
                        slope = utils.find_slope(best_shot_vector, car_to_ball)

                        if forward_flag:
                            # Chute por pulo (baixo)
                            if ball_location[2] <= 300 and slope > 0.0:
                                hits[pair].append(routines.jump_shot(ball_location, intercept_time, best_shot_vector, slope))

                            # Chute aéreo (médio)
                            elif 300 < ball_location[2] < 600 and slope > 1.0:
                                if (ball_location[2] - 250) * 0.14 < agent.me.boost:
                                    hits[pair].append(routines.aerial_shot(ball_location, intercept_time, best_shot_vector, slope))

                            # Aéreo puro (alto)
                            elif ball_location[2] >= 600:
                                aerial_job = routines.aerial(ball_location - 92 * best_shot_vector, intercept_time, True, target=best_shot_vector)
                                if aerial_job.is_viable(agent, agent.time):
                                    hits[pair].append(aerial_job)

                        elif backward_flag and ball_location[2] <= 280 and slope > 0.25:
                            # Chute de ré (pulo)
                            hits[pair].append(routines.jump_shot(ball_location, intercept_time, best_shot_vector, slope, direction=-1))
    return hits

def determine_shot(agent, target, targets, target_count):
    """
    Decide qual o melhor chute disponível e o coloca na pilha de rotinas.
    Prioriza a velocidade de interceptação e a viabilidade.
    """
    if utils.magnitude(agent.ball.velocity) > 0:
        hits = find_hits(agent, targets)

        possible_shots = []
        for i in range(1, target_count + 1):
            target_key = str(i)
            if target_key in hits and len(hits[target_key]) > 0:
                for shot in hits[target_key]:
                    time_delta = shot.intercept_time - agent.time
                    dist_delta = utils.magnitude(agent.me.location - shot.ball_location)
                    avg_speed = dist_delta / time_delta if time_delta > 0 else 0

                    if avg_speed > 500: # Ignora chutes que exigem velocidade muito baixa
                        possible_shots.append(shot)

        if len(possible_shots) > 0:
            # Ordena pelo tempo de interceptação mais rápido
            possible_shots.sort(key=lambda s: s.intercept_time)
            best_shot = possible_shots[0]

            if len(agent.stack) > 0:
                agent.pop()
            agent.push(best_shot)
            return True

    # Se nenhum chute otimizado for encontrado, usa o short_shot básico (drible/empurrão)
    if len(agent.stack) > 0:
        agent.pop()
    agent.push(routines.short_shot(target))
    return False

def intercept_race(agent, target_loc):
    """
    Calcula quem chega primeiro em um ponto: o agente ou o oponente mais próximo.
    Útil para decidir se deve desafiar ou recuar.
    """
    me_dist = utils.magnitude(agent.me.location - target_loc)
    me_speed = utils.magnitude(agent.me.velocity)
    me_eta = me_dist / (me_speed + 500) # Aproximação considerando aceleração

    foes_eta = []
    for foe in agent.foes:
        if not foe.demolished:
            f_dist = utils.magnitude(foe.location - target_loc)
            f_speed = utils.magnitude(foe.velocity)
            foes_eta.append(f_dist / (f_speed + 500))

    min_foe_eta = min(foes_eta) if foes_eta else 9999
    return me_eta < min_foe_eta
