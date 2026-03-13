import math
import numpy as np
from math import atan2
import utils

# Este arquivo contém todas as tarefas mecânicas (rotinas) que o bot pode executar.

# Constantes de Gravidade e Física
GRAVITY = np.array([0, 0, -650], dtype=np.float64)
MAX_SPEED = 2300
BOOST_ACCEL = 1060
THROTTLE_ACCEL = 200 / 3
BOOST_PER_SECOND = 30

# Constantes de Pulo
JUMP_SPEED = 291.667
JUMP_ACC = 1458.3333
JUMP_MIN_DURATION = 0.025
JUMP_MAX_DURATION = 0.2

class atba:
    """Always Towards Ball Agent: Simplesmente dirige em direção à bola."""
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        utils.defaultPD(agent, local_target)
        utils.defaultThrottle(agent, 2300)


class aerial_shot:
    """Rotina para chutar a bola no ar (entre 300 e 600 unidades de altura)."""
    def __init__(self, ball_location, intercept_time, shot_vector, ratio):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.intercept = self.ball_location - (self.shot_vector * 110)
        self.jump_threshold = 600
        self.jump_time = 0
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = utils.cap(raw_time_remaining, 0.01, 10.0)

        car_to_ball = self.ball_location - agent.me.location
        side_of_shot = utils.sign(np.dot(np.cross(self.shot_vector, [0, 0, 1]), car_to_ball))

        car_to_intercept = self.intercept - agent.me.location
        car_to_intercept_perp = np.cross(car_to_intercept, [0, 0, side_of_shot])
        distance_remaining = utils.magnitude(car_to_intercept[:2]) # Flattened distance

        speed_required = distance_remaining / time_remaining
        accel_req = utils.backsolve(self.intercept, agent.me, time_remaining, 0 if self.jump_time == 0 else 325)
        local_accel_req = agent.me.local(accel_req)

        adjustment = utils.angle_between(car_to_intercept, self.shot_vector) * distance_remaining / 1.57
        adjustment *= (utils.cap(self.jump_threshold - accel_req[2], 0.0, self.jump_threshold) / self.jump_threshold)

        final_target = self.intercept + ((utils.normalize(car_to_intercept_perp) * adjustment) if self.jump_time == 0 else 0)

        if utils.in_goal_area(agent):
            final_target[0] = utils.cap(final_target[0], -750, 750)
            final_target[1] = utils.cap(final_target[1], -5050, 5050)

        local_final_target = agent.me.local(final_target - agent.me.location)
        angles = utils.defaultPD(agent, local_final_target)

        if self.jump_time == 0:
            utils.defaultThrottle(agent, speed_required)
            agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
            if accel_req[2] > self.jump_threshold:
                self.jump_time = agent.time
        else:
            time_since_jump = agent.time - self.jump_time
            if agent.me.airborne and utils.magnitude(local_accel_req) * time_remaining > 100:
                angles = utils.defaultPD(agent, local_accel_req)
                if abs(angles[0]) + abs(angles[1]) < 0.5:
                    agent.controller.boost = True

            if self.counter == 0 and (time_since_jump <= 0.2 and local_accel_req[2] > 0):
                agent.controller.jump = True
            elif time_since_jump > 0.2 and self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif local_accel_req[2] > 300 and self.counter == 3:
                agent.controller.jump = True
                self.counter += 1

        if raw_time_remaining < -0.25 or not utils.shot_valid(agent, self):
            agent.pop()
            agent.push(recovery())


class flip:
    """Executa um dodge (pulo duplo com direção) em uma direção local."""
    def __init__(self, vector, cancel=False):
        self.vector = utils.normalize(vector)
        self.pitch = abs(self.vector[0]) * -utils.sign(self.vector[0])
        self.yaw = abs(self.vector[1]) * utils.sign(self.vector[1])
        self.cancel = cancel
        self.time = -1
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            self.time = agent.time
        elapsed = agent.time - self.time

        if elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed >= 0.15 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.pop()
            agent.push(recovery())


class goto:
    """Navega até um alvo estacionário, podendo usar flips para ganhar velocidade."""
    def __init__(self, target, vector=None, direction=1, urgent=False):
        self.target = target
        self.vector = vector
        self.direction = direction
        self.urgent = urgent

    def run(self, agent):
        car_to_target = self.target - agent.me.location
        distance_remaining = utils.magnitude(car_to_target[:2])

        if self.vector is not None:
            side_of_vector = utils.sign(np.dot(np.cross(self.vector, [0, 0, 1]), car_to_target))
            car_to_target_perp = utils.normalize(np.cross(car_to_target, [0, 0, side_of_vector]))
            adjustment = utils.angle_between(car_to_target, self.vector) * distance_remaining / 3.14
            final_target = self.target + (car_to_target_perp * adjustment)
        else:
            final_target = self.target

        if utils.in_goal_area(agent):
            final_target[0] = utils.cap(final_target[0], -750, 750)
            final_target[1] = utils.cap(final_target[1], -5050, 5050)

        local_target = agent.me.local(final_target - agent.me.location)
        angles = utils.defaultPD(agent, local_target, self.direction)
        utils.defaultThrottle(agent, 2300, self.direction)

        agent.controller.boost = True if self.urgent and distance_remaining > 1500 else False
        agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

        velocity = utils.magnitude(agent.me.velocity)

        if distance_remaining < 350:
            agent.pop()
        elif abs(angles[1]) < 0.05 and 600 < velocity < 2150 and distance_remaining / (velocity + 1) > 2.0:
            if agent.me.up[2] < 0.9 or agent.me.airborne:
                agent.push(flip(local_target))
        elif agent.me.airborne:
            agent.push(recovery(self.target))


class jump_shot:
    """Rotina principal para chutar a bola no chão ou logo acima dele."""
    def __init__(self, ball_location, intercept_time, shot_vector, ratio, direction=1):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - (self.shot_vector * 173)
        self.ratio = ratio
        self.direction = direction
        self.jump_threshold = 400
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = utils.cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        side_of_shot = utils.sign(np.dot(np.cross(self.shot_vector, [0, 0, 1]), car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = np.cross(car_to_dodge_point, [0, 0, side_of_shot])
        distance_remaining = utils.magnitude(car_to_dodge_point)

        speed_required = distance_remaining / time_remaining
        accel_req = utils.backsolve(self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650)
        local_accel_req = agent.me.local(accel_req)

        adjustment = utils.angle_between(car_to_dodge_point, self.shot_vector) * distance_remaining / 2.0
        adjustment *= (utils.cap(self.jump_threshold - accel_req[2], 0.0, self.jump_threshold) / self.jump_threshold)

        final_target = self.dodge_point + ((utils.normalize(car_to_dodge_perp) * adjustment) if not self.jumping else 0) + np.array([0, 0, 50])

        if utils.in_goal_area(agent):
            final_target[0] = utils.cap(final_target[0], -750, 750)
            final_target[1] = utils.cap(final_target[1], -5050, 5050)

        local_final_target = agent.me.local(final_target - agent.me.location)
        angles = utils.defaultPD(agent, local_final_target, self.direction)
        utils.defaultThrottle(agent, speed_required, self.direction)

        if not self.jumping:
            if raw_time_remaining <= 0.0 or (speed_required - 2300) * time_remaining > 45 or not utils.shot_valid(agent, self):
                agent.pop()
                if agent.me.airborne: agent.push(recovery())
            elif accel_req[2] > self.jump_threshold:
                self.jumping = True
        else:
            if raw_time_remaining <= -0.9 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.push(recovery())
            elif self.counter == 0 and accel_req[2] > 0 and raw_time_remaining > 0.083:
                agent.controller.jump = True
            elif self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif 0.1 >= raw_time_remaining > -0.9:
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -utils.sign(vector[0])
                    self.y = abs(vector[1]) * utils.sign(vector[1]) * self.direction
                    self.dodging = True
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0


class speed_flip:
    """Mecânica avançada de movimentação: Speed Flip."""
    def __init__(self, target):
        self.target = target
        self.start_time = -1
        self.jump_timer = 0
        self.phase = 0 # 0=Drive, 1=Jump, 2=Dodge, 3=Cancel

    def run(self, agent):
        if self.start_time == -1:
            self.start_time = agent.time

        local_target = agent.me.local(self.target - agent.me.location)
        utils.defaultPD(agent, local_target)

        if self.phase == 0:
            utils.defaultThrottle(agent, 2300)
            if utils.magnitude(agent.me.velocity) > 1050:
                self.phase = 1
                self.jump_timer = agent.time
        elif self.phase == 1:
            jump_elapsed = agent.time - self.jump_timer
            if jump_elapsed < 0.05:
                agent.controller.jump = True
            elif jump_elapsed < 0.1:
                agent.controller.jump = False
            else:
                self.phase = 2
                self.jump_timer = agent.time
        elif self.phase == 2:
            agent.controller.jump = True
            agent.controller.pitch = -1
            angle = atan2(local_target[1], local_target[0])
            agent.controller.yaw = utils.sign(angle) if abs(angle) > 0.1 else 1
            self.phase = 3
            self.jump_timer = agent.time
        elif self.phase == 3:
            cancel_elapsed = agent.time - self.jump_timer
            if cancel_elapsed < 0.6:
                agent.controller.pitch = 1 # Cancelamento
            else:
                agent.pop()
                agent.push(recovery())


class kickoff:
    """Rotina de kickoff simplificada usando speed_flip."""
    def run(self, agent):
        target = agent.ball.location + np.array([0, 200 * utils.side(agent.team), 0])
        local_target = agent.me.local(target - agent.me.location)
        dist = utils.magnitude(local_target)

        if dist > 1200:
            agent.pop()
            agent.push(speed_flip(agent.ball.location))
        elif dist < 650:
            agent.pop()
            agent.push(flip(agent.me.local(agent.foe_goal.location - agent.me.location)))
        else:
            utils.defaultPD(agent, local_target)
            utils.defaultThrottle(agent, 2300)


class recovery:
    """Recupera o controle do carro no ar e orienta para um pouso suave."""
    def __init__(self, target=None):
        self.target = target

    def run(self, agent):
        if self.target is not None:
            local_target = agent.me.local(self.target - agent.me.location)
        else:
            local_target = agent.me.local(agent.me.velocity)

        utils.defaultPD(agent, local_target)
        agent.controller.throttle = 1
        if not agent.me.airborne:
            agent.pop()


class short_shot:
    """Aproximação básica da bola para chute ou drible."""
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball = agent.ball.location - agent.me.location
        distance = utils.magnitude(car_to_ball)
        ball_to_target = utils.normalize(self.target - agent.ball.location)

        relative_velocity = np.dot(utils.normalize(car_to_ball), agent.me.velocity - agent.ball.velocity)
        eta = utils.cap(distance / utils.cap(relative_velocity, 400, 2300), 0.0, 1.5) if relative_velocity != 0 else 1.5

        # Posição de abordagem
        final_target = agent.ball.location - (ball_to_target * 100)

        if utils.in_goal_area(agent):
            final_target[0] = utils.cap(final_target[0], -750, 750)
            final_target[1] = utils.cap(final_target[1], -5050, 5050)

        local_target = agent.me.local(final_target - agent.me.location)
        angles = utils.defaultPD(agent, local_target)
        utils.defaultThrottle(agent, 2300 if distance > 1600 else 2300 - utils.cap(1600 * abs(angles[1]), 0, 2050))

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.pop()
            agent.push(flip(agent.me.local(car_to_ball)))


class aerial:
    """Rotina de voo avançada para interceptar bolas altas."""
    def __init__(self, ball_location, intercept_time, on_ground, target=None):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.target = target
        self.jumping = on_ground
        self.time = -1
        self.jump_time = -1
        self.counter = 0

    def run(self, agent):
        if self.time == -1: self.time = agent.time
        elapsed = agent.time - self.time
        T = self.intercept_time - agent.time

        # Previsão da posição final do carro sem aceleração adicional
        xf = agent.me.location + agent.me.velocity * T + 0.5 * GRAVITY * T**2

        if self.jumping:
            if self.jump_time == -1: self.jump_time = agent.time
            jump_elapsed = agent.time - self.jump_time
            if jump_elapsed < JUMP_MAX_DURATION:
                agent.controller.jump = True
            elif self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif elapsed < 0.3:
                agent.controller.jump = True
            else:
                self.jumping = False

        delta_x = self.ball_location - xf
        direction = utils.normalize(delta_x)

        if utils.magnitude(delta_x) > 50:
            utils.defaultPD(agent, agent.me.local(delta_x))
            if agent.me.forward.dot(direction) > 0.9:
                agent.controller.boost = True
        else:
            if self.target is not None:
                utils.defaultPD(agent, agent.me.local(self.target))
            agent.controller.boost = False

        if T <= 0:
            agent.pop()
            agent.push(recovery())

    def is_viable(self, agent, time):
        """Verifica se o agente tem boost e tempo suficiente para o voo."""
        T = self.intercept_time - time
        if T <= 0: return False

        xf = agent.me.location + agent.me.velocity * T + 0.5 * GRAVITY * T**2
        delta_x = self.ball_location - xf
        req_accel = (2 * utils.magnitude(delta_x)) / (T**2)

        return req_accel < BOOST_ACCEL and agent.me.boost > (req_accel / BOOST_ACCEL) * T * 30
