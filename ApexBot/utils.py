import math
import numpy as np
import rlbot.utils.structures.game_data_struct as game_data_struct
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

# Alias para facilitar a compatibilidade com o renderer do RLBot que exige structs específicas
RLBotVector3 = game_data_struct.Vector3

# --- Classes de Objetos do Jogo ---

class GoslingAgent(BaseAgent):
    """
    Classe base do agente, estendendo BaseAgent do RLBot.
    Gerencia o estado do jogo, objetos e a pilha de rotinas.
    Refatorada para utilizar NumPy em todos os cálculos vetoriais.
    """
    def initialize_agent(self):
        # Listas de carros para aliados e oponentes
        self.friends = []
        self.foes = []
        # Objeto que representa o próprio carro
        self.me = car_object(self.index)

        self.ball = ball_object()
        self.game = game_object()
        # Lista de pads de boost
        self.boosts = []
        # Objetos de gol
        self.friend_goal = goal_object(self.team)
        self.foe_goal = goal_object(not self.team)
        # Pilha de rotinas (mecânicas)
        self.stack = []
        # Tempo de jogo atual
        self.time = 0.0
        # Indica se o agente terminou a inicialização
        self.ready = False
        # Estado do controle enviado ao jogo
        self.controller = SimpleControllerState()
        # Flag indicando se é momento de kickoff
        self.kickoff_flag = False

        self.my_score = 0
        self.foe_score = 0

    def get_ready(self, packet):
        # Prepara os objetos iniciais baseados na informação do campo
        field_info = self.get_field_info()
        for i in range(field_info.num_boosts):
            boost = field_info.boost_pads[i]
            self.boosts.append(boost_object(i, boost.location, boost.is_full_boost))
        self.refresh_player_lists(packet)
        self.ball.update(packet)
        self.ready = True

    def refresh_player_lists(self, packet):
        # Atualiza as listas de aliados e inimigos (útil se jogadores entrarem/saírem)
        self.friends = [car_object(i, packet) for i in range(packet.num_cars) if
                        packet.game_cars[i].team == self.team and i != self.index]
        self.foes = [car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team != self.team]

    def push(self, routine):
        # Adiciona uma rotina ao topo da pilha
        self.stack.append(routine)

    def pop(self):
        # Remove e retorna a rotina do topo da pilha
        if len(self.stack) < 1: return
        return self.stack.pop()

    def line(self, start, end, color=None):
        # Desenha uma linha no mundo 3D para depuração, convertendo NumPy para RLBot Vector3
        color = color if color is not None else [255, 255, 255]
        self.renderer.draw_line_3d(np_to_rlbot(start), np_to_rlbot(end), self.renderer.create_color(255, *color))

    def debug_stack(self):
        # Exibe as rotinas ativas na tela
        white = self.renderer.white()
        for i in range(len(self.stack) - 1, -1, -1):
            text = self.stack[i].__class__.__name__
            self.renderer.draw_string_2d(10 +(250 *self.index), 100 + (50 * (len(self.stack) - i)), 2, 2, text, white)

    def clear(self):
        # Limpa todas as rotinas da pilha
        self.stack = []

    def preprocess(self, packet):
        # Atualiza todos os objetos com os dados do pacote mais recente
        if packet.num_cars != len(self.friends) + len(self.foes) + 1: self.refresh_player_lists(packet)
        for car in self.friends: car.update(packet)
        for car in self.foes: car.update(packet)
        for pad in self.boosts: pad.update(packet)
        self.ball.update(packet)
        self.me.update(packet)
        self.game.update(packet)
        self.time = packet.game_info.seconds_elapsed

        # Se um novo kickoff começa, limpamos a pilha
        if self.kickoff_flag == False and packet.game_info.is_round_active and packet.game_info.is_kickoff_pause:
            self.stack = []

        self.kickoff_flag = packet.game_info.is_round_active and packet.game_info.is_kickoff_pause
        self.my_score = packet.teams[self.team].score
        self.foe_score = packet.teams[1 - self.team].score

    def get_output(self, packet):
        # Reseta o controle e processa o tick do jogo
        self.controller.__init__()
        if not self.ready:
            self.get_ready(packet)
        self.preprocess(packet)

        self.renderer.begin_rendering()
        # Executa a lógica da estratégia (definida na subclasse)
        self.run()
        # Executa a rotina no topo da pilha, se houver
        if len(self.stack) > 0:
            self.stack[-1].run(self)
        self.renderer.end_rendering()
        return self.controller

    def run(self):
        # Método para ser sobrescrito pela estratégia do bot
        pass


class car_object:
    """Representa um carro no jogo usando arrays NumPy para cálculos vetoriais."""
    def __init__(self, index, packet=None):
        self.location = np.array([0, 0, 0], dtype=np.float64)
        self.velocity = np.array([0, 0, 0], dtype=np.float64)
        self.orientation = np.eye(3, dtype=np.float64) # Matriz Identidade padrão
        self.angular_velocity = np.array([0, 0, 0], dtype=np.float64)
        self.demolished = False
        self.airborne = False
        self.supersonic = False
        self.jumped = False
        self.doublejumped = False
        self.team = 0
        self.boost = 0
        self.index = index
        if packet is not None:
            self.team = packet.game_cars[self.index].team
            self.update(packet)

    def local(self, value):
        """Converte um vetor global para o sistema de coordenadas local do carro."""
        return self.orientation.dot(value)

    def update(self, packet):
        """Atualiza os dados do carro a partir do pacote do jogo."""
        car = packet.game_cars[self.index]
        self.location = np.array([car.physics.location.x, car.physics.location.y, car.physics.location.z])
        self.velocity = np.array([car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z])
        self.orientation = orientation_matrix(car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll)

        # Velocidade angular local
        raw_angular = np.array([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
        self.angular_velocity = self.orientation.dot(raw_angular)

        self.demolished = car.is_demolished
        self.airborne = not car.has_wheel_contact
        self.supersonic = car.is_super_sonic
        self.jumped = car.jumped
        self.doublejumped = car.double_jumped
        self.boost = car.boost

    @property
    def forward(self):
        # Vetor unitário apontando para a frente do carro (primeira linha da matriz de orientação)
        return self.orientation[0]

    @property
    def left(self):
        # Vetor unitário apontando para a esquerda do carro (segunda linha)
        return self.orientation[1]

    @property
    def up(self):
        # Vetor unitário apontando para cima do carro (terceira linha)
        return self.orientation[2]


class ball_object:
    """Representa a bola no jogo."""
    def __init__(self):
        self.location = np.array([0, 0, 0], dtype=np.float64)
        self.velocity = np.array([0, 0, 0], dtype=np.float64)
        self.latest_touched_time = 0
        self.latest_touched_team = 0

    def update(self, packet):
        ball = packet.game_ball
        self.location = np.array([ball.physics.location.x, ball.physics.location.y, ball.physics.location.z])
        self.velocity = np.array([ball.physics.velocity.x, ball.physics.velocity.y, ball.physics.velocity.z])
        self.latest_touched_time = ball.latest_touch.time_seconds
        self.latest_touched_team = ball.latest_touch.team


class boost_object:
    """Representa um pad de boost."""
    def __init__(self, index, location, large):
        self.index = index
        # Suporta inicialização por struct do RLBot ou por iterável
        if hasattr(location, 'x'):
            self.location = np.array([location.x, location.y, location.z], dtype=np.float64)
        else:
            self.location = np.array(location, dtype=np.float64)
        self.active = True
        self.large = large

    def update(self, packet):
        self.active = packet.game_boosts[self.index].is_active


class goal_object:
    """Representa o gol de um time."""
    def __init__(self, team):
        side_val = 1 if team == 1 else -1
        self.location = np.array([0, side_val * 5100, 320], dtype=np.float64)
        # Postes do gol (ligeiramente ajustados para maior precisão de chute)
        self.left_post = np.array([side_val * 850, side_val * 5100, 320], dtype=np.float64)
        self.right_post = np.array([-side_val * 850, side_val * 5100, 320], dtype=np.float64)


class game_object:
    """Informações gerais sobre a partida."""
    def __init__(self):
        self.time = 0
        self.time_remaining = 0
        self.overtime = False
        self.round_active = False
        self.kickoff = False
        self.match_ended = False

    def update(self, packet):
        game = packet.game_info
        self.time = game.seconds_elapsed
        self.time_remaining = game.game_time_remaining
        self.overtime = game.is_overtime
        self.round_active = game.is_round_active
        self.kickoff = game.is_kickoff_pause
        self.match_ended = game.is_match_ended

# --- Funções Utilitárias Matemáticas (NumPy) ---

def orientation_matrix(pitch, yaw, roll):
    """Cria uma matriz de orientação (3x3) a partir dos ângulos de Euler."""
    CP = math.cos(pitch)
    SP = math.sin(pitch)
    CY = math.cos(yaw)
    SY = math.sin(yaw)
    CR = math.cos(roll)
    SR = math.sin(roll)

    # Linhas representam os eixos Forward, Left e Up
    matrix = np.array([
        [CP * CY, CP * SY, SP],
        [CY * SP * SR - CR * SY, SY * SP * SR + CR * CY, -CP * SR],
        [-CR * CY * SP - SR * SY, -CR * SY * SP + SR * CY, CP * CR]
    ], dtype=np.float64)
    return matrix

def magnitude(vector):
    """Calcula a magnitude (comprimento) de um vetor."""
    return np.linalg.norm(vector)

def normalize(vector):
    """Retorna o vetor unitário. Se for vetor nulo, retorna o próprio vetor."""
    m = magnitude(vector)
    if m == 0:
        return vector
    return vector / m

def np_to_rlbot(vector):
    """Converte um array NumPy para a struct Vector3 do RLBot (para renderização)."""
    return RLBotVector3(float(vector[0]), float(vector[1]), float(vector[2]))

def backsolve(target, car, time, gravity=650):
    """Calcula a aceleração necessária para atingir um alvo em um tempo determinado."""
    d = target - car.location
    dv = (d / time - car.velocity) / time
    dv[2] += gravity * time # Compensação de gravidade
    return dv

def cap(x, low, high):
    """Limita um valor entre um mínimo e um máximo."""
    return max(min(x, high), low)

def defaultPD(agent, local_target, direction=1.0):
    """
    Controlador Proporcional-Derivativo simples para alinhar o carro a um alvo local.
    """
    # Se estivermos dirigindo para trás, invertemos o alvo local
    local_target = local_target * direction

    # Orientação local do vetor "UP" global
    up = agent.me.local(np.array([0, 0, 1]))

    # Ângulos necessários para Pitch, Yaw e Roll
    target_angles = [
        math.atan2(local_target[2], local_target[0]), # Pitch
        math.atan2(local_target[1], local_target[0]), # Yaw
        math.atan2(up[1], up[2])                       # Roll (manter upright)
    ]

    # Aplica as entradas no controlador baseadas nos ângulos e na velocidade angular
    agent.controller.steer = steerPD(target_angles[1], 0) * direction
    agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 4)
    agent.controller.yaw = steerPD(target_angles[1], -agent.me.angular_velocity[2] / 4)
    agent.controller.roll = steerPD(target_angles[2], agent.me.angular_velocity[0] / 2)

    return target_angles

def defaultThrottle(agent, target_speed, direction=1.0):
    """Acelera o carro até a velocidade desejada usando acelerador e boost."""
    car_speed = agent.me.local(agent.me.velocity)[0]
    t = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((t**2) * sign(t)/1000, -1.0, 1.0)
    # Ativa boost se a diferença de velocidade for alta e estivermos no chão
    agent.controller.boost = True if (t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0) else False
    return car_speed

def steerPD(angle, rate):
    """Função de transferência para o loop PD do esterçamento."""
    return cap(((35 * (angle + rate))**3) / 10, -1.0, 1.0)

def in_field(point, radius):
    """Verifica se um ponto está dentro dos limites do campo de futebol padrão."""
    p = np.abs(point)
    if p[0] > 4080 - radius: return False
    if p[1] > 5900 - radius: return False
    if p[0] > 880 - radius and p[1] > 5105 - radius: return False # Cantos próximos ao gol
    if p[0] > 2650 and p[1] > -p[0] + 8025 - radius: return False # Quinas das paredes
    return True

def angle_between(v1, v2):
    """Calcula o ângulo em radianos entre dois vetores."""
    v1_u = normalize(v1)
    v2_u = normalize(v2)
    return math.acos(cap(np.dot(v1_u, v2_u), -1.0, 1.0))

def sign(x):
    """Retorna o sinal do número (-1, 0 ou 1)."""
    return np.sign(x)

def post_correction(ball_location, left_target, right_target):
    """Ajusta os alvos de chute considerando o raio da bola para garantir que ela entre."""
    ball_radius = 120
    goal_line_perp = np.cross(right_target - left_target, [0, 0, 1])

    left = left_target + (normalize(np.cross(left_target - ball_location, [0, 0, -1])) * ball_radius)
    right = right_target + (normalize(np.cross(right_target - ball_location, [0, 0, 1])) * ball_radius)

    # Verifica se a correção não inverteu os postes (gol impossível)
    swapped = np.dot(normalize(np.cross(left - ball_location, [0, 0, 1])), normalize(right - ball_location)) > -0.1
    return left, right, swapped

def shot_valid(agent, shot, threshold=45):
    """Verifica se a previsão da bola ainda coincide com o alvo do chute."""
    slices = agent.get_ball_prediction_struct().slices
    # Busca binária rápida para encontrar o slice no tempo do intercepto
    soonest = 0
    latest = len(slices) - 1
    while latest - soonest > 1:
        midpoint = (soonest + latest) // 2
        if slices[midpoint].game_seconds > shot.intercept_time:
            latest = midpoint
        else:
            soonest = midpoint

    # Interpolação simples entre slices
    dt = slices[latest].game_seconds - slices[soonest].game_seconds
    t_ratio = (shot.intercept_time - slices[soonest].game_seconds) / dt if dt > 0 else 0

    p1 = np.array([slices[soonest].physics.location.x, slices[soonest].physics.location.y, slices[soonest].physics.location.z])
    p2 = np.array([slices[latest].physics.location.x, slices[latest].physics.location.y, slices[latest].physics.location.z])
    predicted_loc = p1 + (p2 - p1) * t_ratio

    return magnitude(shot.ball_location - predicted_loc) < threshold

def side(team):
    """Retorna o multiplicador de direção baseado no time (-1 para azul, 1 para laranja)."""
    return -1 if team == 0 else 1

def lerp(a, b, t):
    """Interpolação linear."""
    return a + (b - a) * t

def invlerp(a, b, v):
    """Interpolação linear inversa."""
    return (v - a) / (b - a)

def quadratic(a, b, c):
    """Retorna as raízes de uma equação quadrática."""
    inside = math.sqrt((b*b) - (4*a*c))
    if a != 0:
        return (-b + inside)/(2*a), (-b - inside)/(2*a)
    else:
        return -1, -1

def in_goal_area(agent):
    """Verifica se o agente está dentro da área do gol."""
    if abs(agent.me.location[1]) > 5050:
        if abs(agent.me.location[0]) < 880:
            return True
    return False

def find_slope(shot_vector, car_to_target):
    """Calcula a inclinação relativa da abordagem do carro ao vetor de chute."""
    d = np.dot(shot_vector, car_to_target)
    e = abs(np.dot(np.cross(shot_vector, [0, 0, 1]), car_to_target))
    if e == 0: return 10 * sign(d)
    return cap(d / e, -3.0, 3.0)

def detect_demo(agent):
    """Detecta se algum inimigo está tentando uma demolição contra o agente."""
    for car in agent.foes:
        if not car.airborne:
            distance_to_target = magnitude(agent.me.location - car.location)
            velocity = magnitude(car.velocity)
            velocity_needed = 2200 - velocity
            time_boosting_required = velocity_needed / 991.666
            boost_required = 33.3 * time_boosting_required
            distance_required = velocity * time_boosting_required + 0.5 * 991.666 * (time_boosting_required ** 2)

            if velocity > 0:
                time_to_target = distance_to_target / velocity
                aim_point = car.location + time_to_target * car.velocity
                my_future_location = agent.me.location + time_to_target * agent.me.velocity
                can_demo = car.supersonic or (distance_required < distance_to_target and boost_required < car.boost)

                if magnitude(aim_point - my_future_location) < 200 and can_demo:
                    if time_to_target < 0.75:
                        return True, car
    return False, None
