import unittest
import numpy as np
import sys
import os

# Ensure we can import form the current directory
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from routines import atba, goto, recovery, flip
from utils import car_object, ball_object

class MockAgent:
    def __init__(self):
        # Fix lambda to accept self
        self.me = type('obj', (object,), {
            'location': np.zeros(3),
            'velocity': np.zeros(3),
            'orientation': np.eye(3),
            'angular_velocity': np.zeros(3), # Added
            'boost': 33,
            'airborne': False,
            'up': np.array([0,0,1]),
            'local': lambda self, v: v
        })()
        self.ball = type('obj', (object,), {'location': np.array([1000, 0, 0]), 'velocity': np.zeros(3)})()
        self.controller = type('obj', (object,), {'throttle': 0, 'steer': 0, 'pitch': 0, 'yaw': 0, 'roll': 0, 'boost': False, 'handbrake': False, 'jump': False})()
        self.stack = []
        self.time = 0.0
        self.renderer = type('obj', (object,), {'draw_line_3d': lambda *a: None, 'create_color': lambda *a: None})()

    def push(self, routine):
        self.stack.append(routine)

    def pop(self):
        if self.stack: return self.stack.pop()

    def line(self, start, end, color=None):
        pass

class TestRoutines(unittest.TestCase):
    def test_atba(self):
        agent = MockAgent()
        routine = atba()
        routine.run(agent)
        # Check if throttle is applied
        self.assertEqual(agent.controller.throttle, 1.0)

    def test_goto(self):
        agent = MockAgent()
        target = np.array([2000, 2000, 0])
        routine = goto(target)
        routine.run(agent)
        # Should drive towards target
        self.assertEqual(agent.controller.throttle, 1.0)
        # Check steering (mock agent at 0,0,0 facing X, target at 2000,2000,0)
        # Angle is 45 deg left.
        # Steer should be positive (left)
        self.assertTrue(agent.controller.steer > 0)

    def test_recovery(self):
        agent = MockAgent()
        agent.me.airborne = True
        agent.me.velocity = np.array([1000, 0, 0])
        routine = recovery()
        routine.run(agent)
        # Should try to land upright
        # me.up is (0,0,1). Velocity is X.
        # Target up is Z.
        # We are already upright.
        self.assertAlmostEqual(agent.controller.roll, 0, delta=0.1)

    def test_flip(self):
        agent = MockAgent()
        # Flip forward
        vector = np.array([1, 0, 0])
        routine = flip(vector)
        # Run first tick (start)
        routine.run(agent)
        # First tick: Jump = True (elapsed 0 < 0.15)
        self.assertTrue(agent.controller.jump, "Jump should be True on first tick")

        # Advance time 0.2s.
        agent.time += 0.2
        # Run 3 times to exhaust counter (release button)
        routine.run(agent) # counter -> 1
        routine.run(agent) # counter -> 2
        routine.run(agent) # counter -> 3. Jump False.

        self.assertFalse(agent.controller.jump, "Jump should be False during release phase")

        # Advance time to dodge
        agent.time += 0.2 # Total 0.4
        routine.run(agent) # counter is 3, condition fails. Falls to next block.
        # Jump = True, Pitch = -1 (forward) (elapsed 0.4 < 0.9)
        self.assertTrue(agent.controller.jump, "Jump should be True (dodge) after release")
        self.assertEqual(agent.controller.pitch, -1)

if __name__ == '__main__':
    unittest.main()
