import math
import numpy as np
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import Vector3 as RLBotVector3


# --- objects.py content ---

class GoslingAgent(BaseAgent):
    # This is the main object of Gosling Utils. It holds/updates information about the game and runs routines
    # All utils rely on information being structured and accessed the same way as configured in this class
    def initialize_agent(self):
        # A list of cars for both teammates and opponents
        self.friends = []
        self.foes = []
        # This holds the carobject for our agent
        self.me = car_object(self.index)

        self.ball = ball_object()
        self.game = game_object()
        # A list of boosts
        self.boosts = []
        # goals
        self.friend_goal = goal_object(self.team)
        self.foe_goal = goal_object(not self.team)
        # A list that acts as the routines stack
        self.stack = []
        # Game time
        self.time = 0.0
        # Whether or not GoslingAgent has run its get_ready() function
        self.ready = False
        # the controller that is returned to the framework after every tick
        self.controller = SimpleControllerState()
        # a flag that tells us when kickoff is happening
        self.kickoff_flag = False

        self.last_time = 0
        self.my_score = 0
        self.foe_score = 0

    def get_ready(self, packet):
        # Preps all of the objects that will be updated during play
        field_info = self.get_field_info()
        for i in range(field_info.num_boosts):
            boost = field_info.boost_pads[i]
            self.boosts.append(boost_object(i, boost.location, boost.is_full_boost))
        self.refresh_player_lists(packet)
        self.ball.update(packet)
        self.ready = True

    def refresh_player_lists(self, packet):
        # makes new freind/foe lists
        # Useful to keep separate from get_ready because humans can join/leave a match
        self.friends = [car_object(i, packet) for i in range(packet.num_cars) if
                        packet.game_cars[i].team == self.team and i != self.index]
        self.foes = [car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team != self.team]

    def push(self, routine):
        # Shorthand for adding a routine to the stack
        self.stack.append(routine)

    def pop(self):
        # Shorthand for removing a routine from the stack, returns the routine
        if len(self.stack) < 1: return
        return self.stack.pop()

    def line(self, start, end, color=None):
        color = color if color != None else [255, 255, 255]
        # Convert np.array to something compatible with draw_line_3d (requires x, y, z)
        # We use RLBot's Vector3 struct to ensure compatibility with the renderer
        s = RLBotVector3(start[0], start[1], start[2])
        e = RLBotVector3(end[0], end[1], end[2])
        self.renderer.draw_line_3d(s, e, self.renderer.create_color(255, *color))

    def debug_stack(self):
        # Draws the stack on the screen
        white = self.renderer.white()
        for i in range(len(self.stack) - 1, -1, -1):
            text = self.stack[i].__class__.__name__
            self.renderer.draw_string_2d(10 +(250 *self.index), 100 + (50 * (len(self.stack) - i)), 2, 2, text, white)

    def clear(self):
        # Shorthand for clearing the stack of all routines
        self.stack = []

    def preprocess(self, packet):
        # Calling the update functions for all of the objects
        if packet.num_cars != len(self.friends) + len(self.foes) + 1: self.refresh_player_lists(packet)
        for car in self.friends: car.update(packet)
        for car in self.foes: car.update(packet)
        for pad in self.boosts: pad.update(packet)
        self.ball.update(packet)
        self.me.update(packet)
        self.game.update(packet)
        self.time = packet.game_info.seconds_elapsed
        # When a new kickoff begins we empty the stack
        if self.kickoff_flag == False and packet.game_info.is_round_active and packet.game_info.is_kickoff_pause:
            self.stack = []
        # Tells us when to go for kickoff
        self.kickoff_flag = packet.game_info.is_round_active and packet.game_info.is_kickoff_pause
        self.my_score = packet.teams[self.team].score
        if self.team == 0:
            self.foe_score = packet.teams[1].score
        else:
            self.foe_score = packet.teams[0].score

    def get_output(self, packet):
        # Reset controller
        self.controller.__init__()
        # Get ready, then preprocess
        if not self.ready:
            self.get_ready(packet)
        self.preprocess(packet)

        self.renderer.begin_rendering()
        # Run our strategy code
        self.run()
        # run the routine on the end of the stack
        if len(self.stack) > 0:
            self.stack[-1].run(self)
        self.renderer.end_rendering()
        # send our updated controller back to rlbot
        return self.controller

    def run(self):
        # override this with your strategy code
        pass

class car_object:
    # The carObject, and kin, convert the gametickpacket in something a little friendlier to use,
    # and are updated by GoslingAgent as the game runs
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
        # Shorthand for transforming a vector to local coordinates
        return np.dot(value, self.orientation)

    def update(self, packet):
        car = packet.game_cars[self.index]
        self.location = np.array([car.physics.location.x, car.physics.location.y, car.physics.location.z])
        self.velocity = np.array([car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z])

        # Calculate orientation matrix
        CR = math.cos(car.physics.rotation.roll)
        SR = math.sin(car.physics.rotation.roll)
        CP = math.cos(car.physics.rotation.pitch)
        SP = math.sin(car.physics.rotation.pitch)
        CY = math.cos(car.physics.rotation.yaw)
        SY = math.sin(car.physics.rotation.yaw)

        # GoslingUtils Matrix3 structure:
        # Forward, Left, Up
        forward = np.array([CP * CY, CP * SY, SP])
        left = np.array([CY * SP * SR - CR * SY, SY * SP * SR + CR * CY, -CP * SR])
        up = np.array([-CR * CY * SP - SR * SY, -CR * SY * SP + SR * CY, CP * CR])

        self.orientation = np.column_stack((forward, left, up))

        # Angular velocity in local coordinates
        world_ang_vel = np.array([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
        self.angular_velocity = np.dot(world_ang_vel, self.orientation)

        self.demolished = car.is_demolished
        self.airborne = not car.has_wheel_contact
        self.supersonic = car.is_super_sonic
        self.jumped = car.jumped
        self.doublejumped = car.double_jumped
        self.boost = car.boost

    @property
    def forward(self):
        return self.orientation[:, 0]

    @property
    def left(self):
        return self.orientation[:, 1]

    @property
    def up(self):
        return self.orientation[:, 2]


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
        # Fix: handle location being either struct with .x or array
        if hasattr(location, 'x'):
            self.location = np.array([location.x, location.y, location.z])
        else:
            self.location = np.array(location)
        self.active = True
        self.large = large

    def update(self, packet):
        self.active = packet.game_boosts[self.index].is_active


class goal_object:
    # This is a simple object that creates/holds goalpost locations for a given team (for soccer on standard maps only)
    def __init__(self, team):
        team = 1 if team == 1 else -1
        self.location = np.array([0, team * 5100, 320])  # center of goal line
        # Posts are closer to x=750, but this allows the bot to be a little more accurate
        self.left_post = np.array([team * 850, team * 5100, 320])
        self.right_post = np.array([-team * 850, team * 5100, 320])


class game_object:
    # This object holds information about the current match
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

# --- Vector operations helpers ---

def normalize(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return np.zeros_like(vec), 0.0
    return vec / norm, norm

def magnitude(vec):
    return np.linalg.norm(vec)

def distance(vec1, vec2):
    return np.linalg.norm(vec1 - vec2)

def flatten(vec):
    return np.array([vec[0], vec[1], 0])

def cross(v1, v2):
    return np.cross(v1, v2)

def dot(v1, v2):
    return np.dot(v1, v2)

def clamp(vec, start, end):
    # Clamps vector direction between start and end (2D XY plane)
    s, _ = normalize(vec)
    s_flat = flatten(s)
    start_flat = flatten(start)
    end_flat = flatten(end)

    right = np.dot(s_flat, np.cross(end_flat, np.array([0, 0, -1]))) < 0
    left = np.dot(s_flat, np.cross(start_flat, np.array([0, 0, -1]))) > 0

    if np.dot(end_flat, np.cross(start_flat, np.array([0, 0, -1]))) > 0:
        if right and left:
            return vec
    else:
        if right or left:
            return vec

    if np.dot(start_flat, s_flat) < np.dot(end_flat, s_flat):
        return end
    return start

def angle_between(v1, v2):
    v1_norm, n1 = normalize(v1)
    v2_norm, n2 = normalize(v2)
    return math.acos(np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0))

def angle3D(v1, v2):
    return angle_between(v1, v2)

def rotate_2d(vec, angle):
    # Rotates vector by angle in radians in XY plane
    x, y = vec[0], vec[1]
    return np.array([
        math.cos(angle) * x - math.sin(angle) * y,
        math.sin(angle) * x + math.cos(angle) * y,
        vec[2]
    ])

# --- utils.py content ---

def backsolve(target, car, time, gravity=650):
    #Finds the acceleration required for a car to reach a target in a specific amount of time
    d = target - car.location
    dvx = ((d[0]/time) - car.velocity[0]) / time
    dvy = ((d[1]/time) - car.velocity[1]) / time
    dvz = (((d[2]/time) - car.velocity[2]) / time) + (gravity * time)
    return np.array([dvx, dvy, dvz])

def cap(x, low, high):
    #caps/clamps a number between a low and high value
    return max(low, min(high, x))

def defaultPD(agent, local_target, direction = 1.0):
    # points the car towards a given local target.
    # Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target = local_target * direction
    up = agent.me.local(np.array([0, 0, 1]))  # where "up" is in local coordinates
    target_angles = [
        math.atan2(local_target[2], local_target[0]),  # angle required to pitch towards target
        math.atan2(local_target[1], local_target[0]),  # angle required to yaw towards target
        math.atan2(up[1], up[2])]  # angle required to roll upright
    # Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    agent.controller.steer = steerPD(target_angles[1], 0) * direction
    agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 4)
    agent.controller.yaw = steerPD(target_angles[1], -agent.me.angular_velocity[2] / 4)
    agent.controller.roll = steerPD(target_angles[2], agent.me.angular_velocity[0] / 2)
    # Returns the angles, which can be useful for other purposes
    return target_angles

def defaultThrottle(agent, target_speed, direction = 1.0):
    #accelerates the car to a desired speed using throttle and boost
    car_speed = agent.me.local(agent.me.velocity)[0]
    t = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((t**2) * sign(t)/1000, -1.0, 1.0)
    agent.controller.boost = True if t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0 else False
    return car_speed

def in_field(point, radius):
    #determines if a point is inside the standard soccer field
    # point is np array
    point = np.abs(point)
    if point[0] > 4080 - radius:
        return False
    elif point[1] > 5900 - radius:
        return False
    elif point[0] > 880 - radius and point[1] > 5105 - radius:
        return False
    elif point[0] > 2650 and point[1] > -point[0] + 8025 - radius:
        return False
    return True

def find_slope(shot_vector, car_to_target):
    #Finds the slope of your car's position relative to the shot vector (shot vector is y axis)
    d = np.dot(shot_vector, car_to_target)
    e = abs(np.dot(np.cross(shot_vector, np.array([0,0,1])), car_to_target))
    return cap(d / e if e != 0 else 10*sign(d), -3.0, 3.0)

def post_correction(ball_location, left_target, right_target):
    #this function returns target locations that are corrected to account for the ball's radius
    ball_radius = 120
    goal_line_perp = np.cross((right_target - left_target), np.array([0,0,1]))

    left_to_ball_norm, _ = normalize(left_target - ball_location)
    right_to_ball_norm, _ = normalize(right_target - ball_location)

    left = left_target + (np.cross(left_to_ball_norm, np.array([0,0,-1])) * ball_radius)
    right = right_target + (np.cross(right_to_ball_norm, np.array([0,0,1])) * ball_radius)

    left = left_target if np.dot((left-left_target), goal_line_perp) > 0.0 else left
    right = right_target if np.dot((right-right_target), goal_line_perp) > 0.0 else right

    # Check swapped
    left_norm, _ = normalize(left - ball_location)
    right_norm, _ = normalize(right - ball_location)
    swapped = True if np.dot(np.cross(left_norm, np.array([0,0,1])), right_norm) > -0.1 else False

    return left, right, swapped

def quadratic(a,b,c):
    #Returns the two roots of a quadratic
    inside = math.sqrt((b*b) - (4*a*c))
    if a != 0:
        return (-b + inside)/(2*a),(-b - inside)/(2*a)
    else:
        return -1,-1

def shot_valid(agent, shot, threshold = 45):
    #Returns True if the ball is still where the shot anticipates it to be
    slices = agent.get_ball_prediction_struct().slices
    soonest = 0
    latest = len(slices)-1
    while len(slices[soonest:latest+1]) > 2:
        midpoint = (soonest+latest) // 2
        if slices[midpoint].game_seconds > shot.intercept_time:
            latest = midpoint
        else:
            soonest = midpoint
    #preparing to interpolate between the selected slices
    dt = slices[latest].game_seconds - slices[soonest].game_seconds
    time_from_soonest = shot.intercept_time - slices[soonest].game_seconds

    p_soonest = np.array([slices[soonest].physics.location.x, slices[soonest].physics.location.y, slices[soonest].physics.location.z])
    p_latest = np.array([slices[latest].physics.location.x, slices[latest].physics.location.y, slices[latest].physics.location.z])

    slopes = (p_latest - p_soonest) * (1/dt)

    #Determining exactly where the ball will be at the given shot's intercept_time
    predicted_ball_location = p_soonest + (slopes * time_from_soonest)

    #Comparing predicted location with where the shot expects the ball to be
    return np.linalg.norm(shot.ball_location - predicted_ball_location) < threshold

def side(x):
    #returns -1 for blue team and 1 for orange team
    if x == 0:
        return -1
    return 1

def sign(x):
    #returns the sign of a number, -1, 0, +1
    if x < 0.0:
        return -1
    elif x > 0.0:
        return 1
    else:
        return 0.0

def steerPD(angle, rate):
    #A Proportional-Derivative control loop used for defaultPD
    return cap(((35*(angle+rate))**3)/10, -1.0, 1.0)

def lerp(a, b, t):
    #Linearly interpolate from a to b using t
    return (b - a) * t + a

def invlerp(a, b, v):
    #Inverse linear interpolation from a to b with value v
    return (v - a)/(b - a)


def in_goal_area(agent):
    if abs(agent.me.location[1]) > 5050:
        if abs(agent.me.location[0]) < 880:
            return True
    return False

def detect_demo(agent):
    for car in agent.foes:
        if not car.airborne:
            distance_to_target = np.linalg.norm(agent.me.location - car.location)
            velocity = np.linalg.norm(car.velocity)
            velocity_needed = 2200 - velocity
            time_boosting_required = velocity_needed / 991.666
            boost_required = 33.3 * time_boosting_required
            distance_required = velocity * time_boosting_required + 0.5 * 991.666 * (time_boosting_required ** 2)
            time_to_target = distance_to_target / velocity
            aim_point = car.location + time_to_target * velocity
            my_future_location = agent.me.location + time_to_target * np.linalg.norm(agent.me.velocity)
            can_demo = car.supersonic or (distance_required < distance_to_target and boost_required < car.boost)
            if np.linalg.norm(aim_point - my_future_location) and can_demo:
                if time_to_target < 0.75:
                    return False, car #disabled
    return False, None
