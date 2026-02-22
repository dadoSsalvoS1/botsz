import unittest
import numpy as np
import math
from utils import backsolve, steerPD, defaultPD, car_object, sign, cap

# Mock classes for packet
class Vector3Struct:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = x, y, z

class RotatorStruct:
    def __init__(self, pitch=0, yaw=0, roll=0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll

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

class GameTickPacketMock:
    def __init__(self):
        self.game_cars = [CarStruct() for _ in range(10)]
        self.num_cars = 1
        self.game_ball = None # Not needed for car test

class TestUtils(unittest.TestCase):
    def test_sign(self):
        self.assertEqual(sign(10), 1)
        self.assertEqual(sign(-5), -1)
        self.assertEqual(sign(0), 0)

    def test_cap(self):
        self.assertEqual(cap(150, 0, 100), 100)
        self.assertEqual(cap(-50, 0, 100), 0)
        self.assertEqual(cap(50, 0, 100), 50)

    def test_backsolve(self):
        target = np.array([1000, 0, 500])
        car_loc = np.array([0, 0, 0])
        car_vel = np.array([0, 0, 0])
        time = 1.0
        gravity = 650

        # Manually calculate expected
        # d = [1000, 0, 500]
        # dv = d/1 - 0 = [1000, 0, 500]
        # accel = dv/1 = [1000, 0, 500]
        # accel[2] += 650 * 1 = 1150

        class MockCar:
            def __init__(self):
                self.location = car_loc
                self.velocity = car_vel

        result = backsolve(target, MockCar(), time, gravity)

        np.testing.assert_array_almost_equal(result, np.array([1000, 0, 1150]))

    def test_car_object_update(self):
        packet = GameTickPacketMock()
        packet.game_cars[0].physics.location.x = 100
        packet.game_cars[0].physics.location.y = 200
        packet.game_cars[0].physics.location.z = 300
        packet.game_cars[0].physics.rotation.yaw = math.pi / 2 # Facing Left (Y axis)

        car = car_object(0, packet)

        np.testing.assert_array_equal(car.location, np.array([100, 200, 300]))

        # Verify orientation
        # yaw = pi/2 -> cos=0, sin=1.
        # Forward (CP*CY, CP*SY, SP) -> (0, 1, 0) approximately.
        np.testing.assert_array_almost_equal(car.forward, np.array([0, 1, 0]), decimal=5)

    def test_defaultPD(self):
        # Setup a mock agent
        class MockAgent:
            def __init__(self):
                self.me = None
                self.controller = type('obj', (object,), {'steer':0, 'pitch':0, 'yaw':0, 'roll':0})()

        agent = MockAgent()
        packet = GameTickPacketMock()
        packet.game_cars[0].physics.rotation.yaw = 0 # Facing X
        agent.me = car_object(0, packet)

        # Target to the left (Y axis)
        local_target = np.array([0, 100, 0])

        # Should yaw left
        angles = defaultPD(agent, local_target)

        # Check yaw angle
        # atan2(100, 0) = pi/2 = 1.57
        self.assertAlmostEqual(angles[1], 1.57, places=2)

        # Check controller output
        # steerPD(1.57, 0) should be 1.0 (capped)
        self.assertEqual(agent.controller.yaw, 1.0)

if __name__ == '__main__':
    unittest.main()
