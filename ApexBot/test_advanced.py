import unittest
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from routines import jump_shot, aerial_shot, aerial, air_dribble
from utils import backsolve, shot_valid, cap, car_object

# Mocks
class MockAgent:
    def __init__(self):
        self.me = type('obj', (object,), {
            'location': np.zeros(3),
            'velocity': np.zeros(3),
            'orientation': np.eye(3),
            'angular_velocity': np.zeros(3),
            'boost': 33,
            'airborne': False,
            'up': np.array([0,0,1]),
            'forward': np.array([1,0,0]),
            'local': lambda self, v: v
        })()
        self.ball = type('obj', (object,), {'location': np.array([1000, 0, 0]), 'velocity': np.zeros(3)})()
        self.controller = type('obj', (object,), {'throttle': 0, 'steer': 0, 'pitch': 0, 'yaw': 0, 'roll': 0, 'boost': False, 'handbrake': False, 'jump': False})()
        self.stack = []
        self.time = 0.0
        self.renderer = type('obj', (object,), {'draw_line_3d': lambda *a: None, 'create_color': lambda *a: None, 'white': lambda: None, 'draw_string_3d': lambda *a: None})()

        self.get_ball_prediction_struct = lambda: BallPredictionStructMock()

    def push(self, routine):
        self.stack.append(routine)

    def pop(self):
        if self.stack: return self.stack.pop()

    def line(self, *args): pass

class Vector3Struct:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = x, y, z

class PhysicsStruct:
    def __init__(self):
        self.location = Vector3Struct()
        self.velocity = Vector3Struct()
        self.rotation = Vector3Struct()
        self.angular_velocity = Vector3Struct()

class SliceStruct:
    def __init__(self, time):
        self.game_seconds = time
        self.physics = PhysicsStruct()

class BallPredictionStructMock:
    def __init__(self):
        self.slices = [SliceStruct(i * 0.1) for i in range(60)]
        self.num_slices = 60

class TestAdvancedRoutines(unittest.TestCase):
    def test_air_dribble_run(self):
        agent = MockAgent()
        agent.me.airborne = True
        agent.me.velocity = np.array([0,0,100])
        agent.ball.location = np.array([0,0,200])
        agent.ball.velocity = np.array([0,0,100])

        dribble = air_dribble()
        dribble.run(agent)

        # Should boost if falling relative to ball
        # Ball vel 100, car vel 100.
        # Check logic: if car.vel.z < ball.vel.z + 50 (150). Yes 100 < 150.
        self.assertTrue(agent.controller.boost)

    def test_aerial_fast_run(self):
        agent = MockAgent()
        ball_loc = np.array([0, 0, 800]) # High ball
        # Aerial needs time > 0
        shot = aerial(ball_loc, 2.0, True)

        # Run
        shot.run(agent)

        # Should jump (since on_ground=True)
        self.assertTrue(agent.controller.jump)
        # Should pitch back for fast aerial
        self.assertEqual(agent.controller.pitch, 1.0)
        # Should boost
        self.assertTrue(agent.controller.boost)

if __name__ == '__main__':
    unittest.main()
