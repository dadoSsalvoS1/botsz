import math
import numpy as np

try:
    from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
    from rlbot.utils.structures.game_data_struct import GameTickPacket
except ImportError:
    # Mock classes for testing in environments where rlbot is broken (e.g. Python 3.12 vs flatbuffers)
    class BaseAgent:
        def __init__(self, name, team, index):
            self.name = name
            self.team = team
            self.index = index
            self.renderer = type('obj', (object,), {'draw_line_3d': lambda *a: None, 'create_color': lambda *a: None, 'draw_string_2d': lambda *a: None, 'white': lambda *a: None, 'begin_rendering': lambda *a: None, 'end_rendering': lambda *a: None, 'draw_string_3d': lambda *a: None})()

    class SimpleControllerState:
        def __init__(self):
            self.steer = 0
            self.throttle = 0
            self.pitch = 0
            self.yaw = 0
            self.roll = 0
            self.jump = False
            self.boost = False
            self.handbrake = False
            self.use_item = False

    class GameTickPacket:
        pass

class GoslingAgent(BaseAgent):
    def initialize_agent(self):
        self.friends = []
        self.foes = []
        self.me = car_object(self.index)
        self.ball = ball_object()
        self.game = game_object()
        self.boosts = []
        self.friend_goal = goal_object(self.team)
        self.foe_goal = goal_object(not self.team)
        self.stack = []
        self.time = 0.0
        self.ready = False
        self.controller = SimpleControllerState()
        self.kickoff_flag = False
        self.last_time = 0
        self.my_score = 0
        self.foe_score = 0

    def get_ready(self, packet):
        field_info = self.get_field_info()
        self.boosts = []
        for i in range(field_info.num_boosts):
            boost = field_info.boost_pads[i]
            # Convert boost location to numpy array immediately
            location = np.array([boost.location.x, boost.location.y, boost.location.z])
            self.boosts.append(boost_object(i, location, boost.is_full_boost))
        self.refresh_player_lists(packet)
        self.ball.update(packet)
        self.ready = True

    def refresh_player_lists(self, packet):
        self.friends = [car_object(i, packet) for i in range(packet.num_cars) if
                        packet.game_cars[i].team == self.team and i != self.index]
        self.foes = [car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team != self.team]

    def push(self, routine):
        self.stack.append(routine)

    def pop(self):
        if len(self.stack) < 1: return
        return self.stack.pop()

    def line(self, start, end, color=None):
        color = color if color is not None else [255, 255, 255]
        # Start and end should be numpy arrays, but draw_line_3d expects separate x,y,z arguments or a list-like structure
        # We can pass them as lists or tuples
        self.renderer.draw_line_3d(start.tolist(), end.tolist(), self.renderer.create_color(255, *color))

    def debug_stack(self):
        white = self.renderer.white()
        for i in range(len(self.stack) - 1, -1, -1):
            text = self.stack[i].__class__.__name__
            self.renderer.draw_string_2d(10 +(250 *self.index), 100 + (50 * (len(self.stack) - i)), 2, 2, text, white)

    def clear(self):
        self.stack = []

    def preprocess(self, packet):
        if packet.num_cars != len(self.friends) + len(self.foes) + 1:
            self.refresh_player_lists(packet)

        for car in self.friends: car.update(packet)
        for car in self.foes: car.update(packet)
        for pad in self.boosts: pad.update(packet)

        self.ball.update(packet)
        self.me.update(packet)
        self.game.update(packet)

        self.time = packet.game_info.seconds_elapsed

        if self.kickoff_flag == False and packet.game_info.is_round_active and packet.game_info.is_kickoff_pause:
            self.stack = []

        self.kickoff_flag = packet.game_info.is_round_active and packet.game_info.is_kickoff_pause
        self.my_score = packet.teams[self.team].score
        if self.team == 0:
            self.foe_score = packet.teams[1].score
        else:
            self.foe_score = packet.teams[0].score

    def get_output(self, packet):
        self.controller.__init__()
        if not self.ready:
            self.get_ready(packet)
        self.preprocess(packet)

        self.renderer.begin_rendering()
        self.run()
        if len(self.stack) > 0:
            self.stack[-1].run(self)
        self.renderer.end_rendering()
        return self.controller

    def run(self):
        pass


class car_object:
    def __init__(self, index, packet=None):
        self.location = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3) # Identity matrix
        self.angular_velocity = np.zeros(3)
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
        # Transform vector 'value' (world coords) to local coords relative to car
        # This is typically done by dot product with the orientation matrix
        # GoslingUtils `local` seems to do: self.orientation.dot(value)
        # Assuming orientation rows are forward, left, up (or similar basis)
        # We need to verify the matrix construction.
        # GoslingUtils Matrix3:
        # forward, left, up = self.data
        # So it's a list of 3 vectors.
        # dot(vector) -> Vector3(forward.dot(v), left.dot(v), up.dot(v))
        # This is equivalent to multiplying the matrix by the vector if rows are forward, left, up.
        return np.dot(self.orientation, value) # dot product of (3,3) and (3,) -> (3,)

    def update(self, packet):
        car = packet.game_cars[self.index]
        self.location = np.array([car.physics.location.x, car.physics.location.y, car.physics.location.z])
        self.velocity = np.array([car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z])

        # Calculate orientation matrix
        pitch = car.physics.rotation.pitch
        yaw = car.physics.rotation.yaw
        roll = car.physics.rotation.roll

        # Math from GoslingUtils Matrix3
        CP = math.cos(pitch)
        SP = math.sin(pitch)
        CY = math.cos(yaw)
        SY = math.sin(yaw)
        CR = math.cos(roll)
        SR = math.sin(roll)

        forward = np.array([CP * CY, CP * SY, SP])
        left = np.array([CY * SP * SR - CR * SY, SY * SP * SR + CR * CY, -CP * SR])
        up = np.array([-CR * CY * SP - SR * SY, -CR * SY * SP + SR * CY, CP * CR])

        self.orientation = np.array([forward, left, up])

        angular_velocity_world = np.array([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
        self.angular_velocity = np.dot(self.orientation, angular_velocity_world)

        self.demolished = car.is_demolished
        self.airborne = not car.has_wheel_contact
        self.supersonic = car.is_super_sonic
        self.jumped = car.jumped
        self.doublejumped = car.double_jumped
        self.boost = car.boost

    @property
    def forward(self):
        return self.orientation[0]

    @property
    def left(self):
        return self.orientation[1]

    @property
    def up(self):
        return self.orientation[2]


class ball_object:
    def __init__(self):
        self.location = np.zeros(3)
        self.velocity = np.zeros(3)
        self.latest_touched_time = 0
        self.latest_touched_team = 0

    def update(self, packet):
        ball = packet.game_ball
        self.location = np.array([ball.physics.location.x, ball.physics.location.y, ball.physics.location.z])
        self.velocity = np.array([ball.physics.velocity.x, ball.physics.velocity.y, ball.physics.velocity.z])
        self.latest_touched_time = ball.latest_touch.time_seconds
        self.latest_touched_team = ball.latest_touch.team


class boost_object:
    def __init__(self, index, location, large):
        self.index = index
        self.location = location # Already numpy array
        self.active = True
        self.large = large

    def update(self, packet):
        self.active = packet.game_boosts[self.index].is_active


class goal_object:
    def __init__(self, team):
        team_sign = 1 if team == 1 else -1
        self.location = np.array([0, team_sign * 5100, 320])
        self.left_post = np.array([team_sign * 850, team_sign * 5100, 320])
        self.right_post = np.array([-team_sign * 850, team_sign * 5100, 320])


class game_object:
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


# --- Helper Functions ---

def backsolve(target, car, time, gravity=650):
    # Finds the acceleration required for a car to reach a target in a specific amount of time
    # target: np.array, car: car_object, time: float
    d = target - car.location
    dv = (d / time) - car.velocity
    accel = dv / time
    accel[2] += gravity * time
    return accel # Returns np.array([ax, ay, az])

def cap(x, low, high):
    if x < low: return low
    if x > high: return high
    return x

def defaultPD(agent, local_target, direction=1.0):
    # Points the car towards a given local target
    local_target = local_target * direction

    # We need to find angles to rotate
    # Pitch: Angle in XZ plane (up/down)
    pitch_angle = math.atan2(local_target[2], local_target[0])

    # Yaw: Angle in XY plane (left/right)
    yaw_angle = math.atan2(local_target[1], local_target[0])

    # Roll: We want to be upright relative to world up.
    # Convert world up (0,0,1) to local coordinates
    # agent.me.orientation is [forward, left, up]
    # world_up local = orientation . world_up = [forward.z, left.z, up.z]
    # But wait, local() does `dot(orientation, world_up)`.
    # orientation matrix has rows as forward, left, up.
    # world_up is [0,0,1].
    # So dot product is indeed [forward[2], left[2], up[2]].
    up_local = agent.me.local(np.array([0, 0, 1]))

    roll_angle = math.atan2(up_local[1], up_local[2])

    agent.controller.steer = steerPD(yaw_angle, 0) * direction
    agent.controller.pitch = steerPD(pitch_angle, agent.me.angular_velocity[1] / 4)
    agent.controller.yaw = steerPD(yaw_angle, -agent.me.angular_velocity[2] / 4)
    agent.controller.roll = steerPD(roll_angle, agent.me.angular_velocity[0] / 2)

    return np.array([pitch_angle, yaw_angle, roll_angle])

def defaultThrottle(agent, target_speed, direction=1.0):
    car_speed = agent.me.local(agent.me.velocity)[0]
    t = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((t**2) * sign(t)/1000, -1.0, 1.0)
    agent.controller.boost = True if t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0 else False
    return car_speed

def in_field(point, radius):
    point = np.abs(point)
    if point[0] > 4080 - radius: return False
    elif point[1] > 5900 - radius: return False
    elif point[0] > 880 - radius and point[1] > 5105 - radius: return False
    elif point[0] > 2650 and point[1] > -point[0] + 8025 - radius: return False
    return True

def find_slope(shot_vector, car_to_target):
    d = np.dot(shot_vector, car_to_target)
    # Cross product of shot_vector and Z-axis (0,0,1)
    # shot_vector is (3,). (0,0,1) is (3,)
    cross_z = np.cross(shot_vector, np.array([0,0,1]))
    e = abs(np.dot(cross_z, car_to_target))
    return cap(d / e if e != 0 else 10*sign(d), -3.0, 3.0)

def post_correction(ball_location, left_target, right_target):
    ball_radius = 120
    # Goal line perpendicular
    goal_line_vec = right_target - left_target
    goal_line_perp = np.cross(goal_line_vec, np.array([0,0,1]))

    # Left correction
    left_to_ball = (ball_location - left_target)
    left_to_ball_norm = left_to_ball / np.linalg.norm(left_to_ball)
    left_correction = np.cross(left_to_ball_norm, np.array([0,0,-1])) * ball_radius
    left = left_target + left_correction

    # Right correction
    right_to_ball = (ball_location - right_target)
    right_to_ball_norm = right_to_ball / np.linalg.norm(right_to_ball)
    right_correction = np.cross(right_to_ball_norm, np.array([0,0,1])) * ball_radius
    right = right_target + right_correction

    # Verify side of goal line
    if np.dot(left - left_target, goal_line_perp) > 0.0:
        left = left_target
    if np.dot(right - right_target, goal_line_perp) > 0.0:
        right = right_target

    swapped = True if np.dot(np.cross((left - ball_location)/np.linalg.norm(left-ball_location), np.array([0,0,1])), (right - ball_location)/np.linalg.norm(right-ball_location)) > -0.1 else False

    return left, right, swapped

def quadratic(a, b, c):
    inside = math.sqrt((b*b) - (4*a*c))
    if a != 0:
        return (-b + inside)/(2*a), (-b - inside)/(2*a)
    else:
        return -1, -1

def shot_valid(agent, shot, threshold=45):
    slices = agent.get_ball_prediction_struct().slices
    soonest = 0
    latest = len(slices) - 1

    # Binary search for the slice closest to intercept time
    while latest - soonest > 1:
        midpoint = (soonest + latest) // 2
        if slices[midpoint].game_seconds > shot.intercept_time:
            latest = midpoint
        else:
            soonest = midpoint

    # Interpolate
    dt = slices[latest].game_seconds - slices[soonest].game_seconds
    if dt == 0: return False # Should not happen

    time_from_soonest = shot.intercept_time - slices[soonest].game_seconds

    loc_soonest = np.array([slices[soonest].physics.location.x, slices[soonest].physics.location.y, slices[soonest].physics.location.z])
    loc_latest = np.array([slices[latest].physics.location.x, slices[latest].physics.location.y, slices[latest].physics.location.z])

    slopes = (loc_latest - loc_soonest) * (1/dt)
    predicted_ball_location = loc_soonest + (slopes * time_from_soonest)

    return np.linalg.norm(shot.ball_location - predicted_ball_location) < threshold

def side(x):
    if x == 0: return -1
    return 1

def sign(x):
    if x < 0.0: return -1
    elif x > 0.0: return 1
    else: return 0.0

def steerPD(angle, rate):
    return cap(((35*(angle+rate))**3)/10, -1.0, 1.0)

def lerp(a, b, t):
    return (b - a) * t + a

def invlerp(a, b, v):
    return (v - a) / (b - a)

def in_goal_area(agent):
    if abs(agent.me.location[1]) > 5050:
        if abs(agent.me.location[0]) < 880:
            return True
    return False
