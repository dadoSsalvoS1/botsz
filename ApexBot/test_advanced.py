import unittest
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from routines import jump_shot, aerial_shot, aerial, air_dribble, short_shot, wall_shot
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
        # Car behind ball (X axis)
        agent.me.location = np.array([0,0,200])
        agent.me.velocity = np.array([50,0,0])

        agent.ball.location = np.array([100,0,200])
        agent.ball.velocity = np.array([100,0,0])

        dribble = air_dribble()
        dribble.run(agent)

        # Car slower than ball, should throttle up
        self.assertEqual(agent.controller.throttle, 1.0)
        self.assertTrue(agent.controller.boost)

    def test_short_shot_hit(self):
        agent = MockAgent()
        # Close to ball
        agent.ball.location = np.array([100,0,0])
        shot = short_shot(np.array([2000,0,0])) # Target ahead
        shot.run(agent)

        # Should flip
        self.assertTrue(agent.stack[-1].__class__.__name__ == 'flip')

    def test_wall_shot_drive(self):
        agent = MockAgent()
        agent.me.location = np.array([3900, 0, 10]) # Near wall, on ground
        agent.ball.location = np.array([3900, 0, 500]) # Ball up wall

        shot = wall_shot(np.array([0, 5000, 0]))
        shot.run(agent)

        # Should drive (atba/goto logic inline or fallback)
        # Logic says: if on ground (<20), pop and push short_shot fallback.
        # Check stack
        self.assertEqual(agent.stack[-1].__class__.__name__, 'short_shot')

if __name__ == '__main__':
    unittest.main()
