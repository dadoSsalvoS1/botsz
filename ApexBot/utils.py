import math
import numpy as np
import rlbot.utils.structures.game_data_struct as game_data_struct
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

# --- Helper Functions ---

def normalize(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm

def cap(x, low, high):
    return np.clip(x, low, high)

def orientation_matrix(pitch, yaw, roll):
    # Calculate trig functions
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cr = math.cos(roll)
    sr = math.sin(roll)

    # Calculate rotation matrix
    # Forward, Left, Up
    # This matches the GoslingUtils Matrix3 structure
    # Row 0: Forward
    # Row 1: Left
    # Row 2: Up
    theta = np.array([
        [cp*cy, cp*sy, sp],
        [cy*sp*sr - cr*sy, sy*sp*sr + cr*cy, -cp*sr],
        [-cr*cy*sp - sr*sy, -cr*sy*sp + sr*cy, cp*cr]
    ])
    return theta

def local(car, vec_global):
    # Transform global vector to local coordinates
    # local = orientation . global
    # Since orientation rows are the axes, dot product projects onto them.
    return np.dot(car.orientation, vec_global)

def world(car, vec_local):
    # Transform local vector to global coordinates
    # global = orientation.T . local (since orientation is orthogonal)
    return np.dot(car.orientation.T, vec_local)

def steerPD(angle, rate):
    return cap(((35*(angle+rate))**3)/10, -1.0, 1.0)

def backsolve(target, car, time, gravity=650):
    d = target - car.location
    dvx = ((d[0]/time) - car.velocity[0]) / time
    dvy = ((d[1]/time) - car.velocity[1]) / time
    dvz = (((d[2]/time) - car.velocity[2]) / time) + (gravity * time)
    return np.array([dvx, dvy, dvz])

def defaultPD(agent, local_target, direction=1.0):
    local_target *= direction
    up = local(agent.me, np.array([0, 0, 1]))
    target_angles = [
        math.atan2(local_target[2], local_target[0]),
        math.atan2(local_target[1], local_target[0]),
        math.atan2(up[1], up[2])
    ]
    agent.controller.steer = steerPD(target_angles[1], 0) * direction
    agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1]/4)
    agent.controller.yaw = steerPD(target_angles[1], -agent.me.angular_velocity[2]/4)
    agent.controller.roll = steerPD(target_angles[2], agent.me.angular_velocity[0]/2)
    return target_angles

def defaultThrottle(agent, target_speed, direction=1.0):
    car_speed = local(agent.me, agent.me.velocity)[0]
    t = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((t**2) * np.sign(t)/1000, -1.0, 1.0)
    agent.controller.boost = True if t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0 else False
    return car_speed

def in_field(point, radius):
    point = np.abs(point)
    if point[0] > 4080 - radius: return False
    if point[1] > 5900 - radius: return False
    if point[0] > 880 - radius and point[1] > 5105 - radius: return False
    if point[0] > 2650 and point[1] > -point[0] + 8025 - radius: return False
    return True

def find_slope(shot_vector, car_to_target):
    d = np.dot(shot_vector, car_to_target)
    e = abs(np.dot(np.cross(shot_vector, np.array([0, 0, 1])), car_to_target))
    return cap(d / e if e != 0 else 10*np.sign(d), -3.0, 3.0)

def post_correction(ball_location, left_target, right_target):
    ball_radius = 120
    goal_line_perp = np.cross(right_target - left_target, np.array([0, 0, 1]))

    left_to_ball = normalize(left_target - ball_location)
    right_to_ball = normalize(right_target - ball_location)

    left = left_target + np.cross(left_to_ball, np.array([0, 0, -1])) * ball_radius
    right = right_target + np.cross(right_to_ball, np.array([0, 0, 1])) * ball_radius

    left = left_target if np.dot(left - left_target, goal_line_perp) > 0.0 else left
    right = right_target if np.dot(right - right_target, goal_line_perp) > 0.0 else right

    swapped = True if np.dot(np.cross(normalize(left - ball_location), np.array([0, 0, 1])), normalize(right - ball_location)) > -0.1 else False
    return left, right, swapped

def side(x):
    if x == 0: return -1
    return 1

def sign(x):
    return np.sign(x)

def clamp_vector(v, start, end):
    # Vector3.clamp logic replication
    # Forces vector v between start and end in 2D (xy plane) clockwise
    s = normalize(v)
    right = np.dot(s, np.cross(end, np.array([0, 0, -1]))) < 0
    left = np.dot(s, np.cross(start, np.array([0, 0, -1]))) > 0

    if (np.dot(end, np.cross(start, np.array([0, 0, -1]))) > 0):
        if right and left: return v
    else:
        if right or left: return v

    if np.dot(start, s) < np.dot(end, s):
        return end
    return start

def angle_between(v1, v2):
    # 2D angle between vectors (flattened)
    v1_flat = normalize(np.array([v1[0], v1[1], 0]))
    v2_flat = normalize(np.array([v2[0], v2[1], 0]))
    return math.acos(cap(np.dot(v1_flat, v2_flat), -1.0, 1.0))

def angle3D(v1, v2):
    v1_n = normalize(v1)
    v2_n = normalize(v2)
    return math.acos(cap(np.dot(v1_n, v2_n), -1.0, 1.0))

# --- Objects ---

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
        for i in range(field_info.num_boosts):
            boost = field_info.boost_pads[i]
            # Convert vector3 to numpy array
            loc = np.array([boost.location.x, boost.location.y, boost.location.z])
            self.boosts.append(boost_object(i, loc, boost.is_full_boost))
        self.refresh_player_lists(packet)
        self.ball.update(packet)
        self.ready = True

    def refresh_player_lists(self, packet):
        self.friends = [car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team == self.team and i != self.index]
        self.foes = [car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team != self.team]

    def push(self, routine):
        self.stack.append(routine)

    def pop(self):
        if len(self.stack) < 1: return
        return self.stack.pop()

    def line(self, start, end, color=None):
        color = color if color != None else [255, 255, 255]
        # start and end are numpy arrays, renderer expects rlbot objects usually, but let's check
        # The renderer methods usually accept simple lists/tuples
        self.renderer.draw_line_3d(start.tolist(), end.tolist(), self.renderer.create_color(255, *color))

    def debug_stack(self):
        white = self.renderer.white()
        for i in range(len(self.stack) - 1, -1, -1):
            text = self.stack[i].__class__.__name__
            self.renderer.draw_string_2d(10 + (250 * self.index), 100 + (50 * (len(self.stack) - i)), 2, 2, text, white)

    def clear(self):
        self.stack = []

    def preprocess(self, packet):
        if packet.num_cars != len(self.friends) + len(self.foes) + 1: self.refresh_player_lists(packet)
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
        self.orientation = np.eye(3)
        self.velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.demolished = False
        self.airborne = False
        self.supersonic = False
        self.jumped = False
        self.doublejumped = False
        self.team = 0
        self.boost = 0
        self.index = index
        if packet != None:
            self.team = packet.game_cars[self.index].team
            self.update(packet)

    def local(self, value):
        return local(self, value)

    def update(self, packet):
        car = packet.game_cars[self.index]
        self.location = np.array([car.physics.location.x, car.physics.location.y, car.physics.location.z])
        self.velocity = np.array([car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z])
        self.orientation = orientation_matrix(car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll)
        # Angular velocity in local coordinates? The original code had:
        # self.orientation.dot([global_ang_vel])
        # So yes, local angular velocity
        global_ang_vel = np.array([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
        self.angular_velocity = np.dot(self.orientation, global_ang_vel)

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
        self.location = location # Already numpy array from get_ready
        self.active = True
        self.large = large

    def update(self, packet):
        self.active = packet.game_boosts[self.index].is_active

class goal_object:
    def __init__(self, team):
        team = 1 if team == 1 else -1
        self.location = np.array([0, team * 5100, 320])
        self.left_post = np.array([team * 850, team * 5100, 320])
        self.right_post = np.array([-team * 850, team * 5100, 320])

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
