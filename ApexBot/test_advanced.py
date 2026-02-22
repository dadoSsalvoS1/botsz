import unittest
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from routines import jump_shot, aerial_shot, aerial
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
            'local': lambda self, v: v # Simplify local transform
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
    def test_jump_shot_init(self):
        # Ensure init doesn't crash with zero vectors
        ball_loc = np.array([0, 0, 0])
        shot_vec = np.array([1, 0, 0])
        intercept_time = 1.0
        shot = jump_shot(ball_loc, intercept_time, shot_vec, 1.0)
        self.assertIsNotNone(shot)

    def test_jump_shot_run(self):
        agent = MockAgent()
        shot_vec = np.array([1, 0, 0])
        ball_loc = np.array([1000, 0, 0])
        intercept_time = 1.0 # 1s in future

        shot = jump_shot(ball_loc, intercept_time, shot_vec, 1.0)

        # Run
        shot.run(agent)

        # Should drive forward
        self.assertEqual(agent.controller.throttle, 1.0)

    def test_aerial_init(self):
        ball_loc = np.array([0, 0, 500])
        shot = aerial(ball_loc, 2.0, True)
        self.assertIsNotNone(shot)

    def test_aerial_run(self):
        agent = MockAgent()
        ball_loc = np.array([0, 0, 500])
        # Aerial needs time > 0
        shot = aerial(ball_loc, 2.0, True)

        # Run
        shot.run(agent)

        # Should jump (since on_ground=True)
        self.assertTrue(agent.controller.jump)

if __name__ == '__main__':
    unittest.main()
