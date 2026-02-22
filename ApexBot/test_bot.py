import unittest
import numpy as np
import sys
import os

# Ensure we can import form the current directory
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from bot import ApexBot
from utils import car_object, ball_object

# Mocks for packets
class Vector3Struct:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = float(x), float(y), float(z)

class RotatorStruct:
    def __init__(self, pitch=0, yaw=0, roll=0):
        self.pitch, self.yaw, self.roll = float(pitch), float(yaw), float(roll)

class PhysicsStruct:
    def __init__(self):
        self.location = Vector3Struct()
        self.velocity = Vector3Struct()
        self.rotation = RotatorStruct()
        self.angular_velocity = Vector3Struct()

class CarStruct:
    def __init__(self):
        self.physics = PhysicsStruct()
        self.is_demolished = False
        self.has_wheel_contact = True
        self.is_super_sonic = False
        self.jumped = False
        self.double_jumped = False
        self.team = 0
        self.boost = 33
        self.name = "TestBot"

class BallStruct:
    def __init__(self):
        self.physics = PhysicsStruct()
        self.latest_touch = type('obj', (object,), {'time_seconds': 0, 'team': 0})()

class GameInfoStruct:
    def __init__(self):
        self.seconds_elapsed = 0
        self.game_time_remaining = 300
        self.is_overtime = False
        self.is_round_active = True
        self.is_kickoff_pause = False
        self.is_match_ended = False

class TeamStruct:
    def __init__(self):
        self.score = 0

class BoostStateStruct:
    def __init__(self):
        self.is_active = True
        self.timer = 0.0

class GameTickPacketMock:
    def __init__(self):
        self.game_cars = [CarStruct() for _ in range(10)]
        self.num_cars = 2
        self.game_ball = BallStruct()
        self.game_info = GameInfoStruct()
        self.teams = [TeamStruct(), TeamStruct()]
        self.game_boosts = [BoostStateStruct() for _ in range(50)] # Enough for standard map

class SliceStruct:
    def __init__(self, time):
        self.game_seconds = time
        self.physics = PhysicsStruct()

class BallPredictionStructMock:
    def __init__(self):
        self.slices = [SliceStruct(i * 0.1) for i in range(60)] # 6 seconds prediction
        self.num_slices = 60

class BoostPadStruct:
    def __init__(self):
        self.location = Vector3Struct(0,0,0)
        self.is_full_boost = True

class FieldInfoStructMock:
    def __init__(self):
        self.num_boosts = 10
        self.boost_pads = [BoostPadStruct() for _ in range(10)]

class TestableApexBot(ApexBot):
    def get_ball_prediction_struct(self):
        return BallPredictionStructMock()

    def get_field_info(self):
        return FieldInfoStructMock()

class TestBot(unittest.TestCase):
    def test_run(self):
        # Index 0, Team 0
        bot = TestableApexBot("ApexBot", 0, 0)
        bot.initialize_agent()

        packet = GameTickPacketMock()
        # Set positions
        packet.game_cars[0].physics.location = Vector3Struct(0, 0, 0)
        packet.game_ball.physics.location = Vector3Struct(1000, 0, 0)
        packet.game_info.is_kickoff_pause = False

        # Run one tick
        controller = bot.get_output(packet)

        # Check if controller is valid
        self.assertIsNotNone(controller)
        # Should drive towards ball (short_shot or goto)
        # short_shot sets throttle
        self.assertEqual(controller.throttle, 1.0)

        # Run kickoff
        packet.game_info.is_kickoff_pause = True
        controller = bot.get_output(packet)
        # Should speed flip
        # Speed flip first phase (drive) sets throttle
        self.assertEqual(controller.throttle, 1.0)

if __name__ == '__main__':
    unittest.main()
