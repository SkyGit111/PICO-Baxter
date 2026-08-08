import importlib.util
import math
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


def load_teleop_module():
    baxter_interface = types.ModuleType("baxter_interface")
    baxter_interface.CHECK_VERSION = object()

    rospy = types.ModuleType("rospy")
    rospy.is_shutdown = lambda: False
    rospy.ServiceException = RuntimeError
    rospy.Time = SimpleNamespace(now=lambda: 123.0)

    class SolvePositionIKRequest:
        SEED_USER = 1
        SEED_CURRENT = 2

        def __init__(self):
            self.pose_stamp = []
            self.seed_angles = []
            self.seed_mode = None

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )

    class JointState:
        def __init__(self):
            self.name = []
            self.position = []

    baxter_core_msgs = types.ModuleType("baxter_core_msgs")
    baxter_core_srv = types.ModuleType("baxter_core_msgs.srv")
    baxter_core_srv.SolvePositionIK = object
    baxter_core_srv.SolvePositionIKRequest = SolvePositionIKRequest
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = PoseStamped
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = JointState

    stubs = {
        "baxter_interface": baxter_interface,
        "rospy": rospy,
        "baxter_core_msgs": baxter_core_msgs,
        "baxter_core_msgs.srv": baxter_core_srv,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    }
    for name, module in stubs.items():
        sys.modules[name] = module

    module_name = "baxter_precision_teleop_under_test"
    path = Path(__file__).resolve().parents[1] / "baxter_precision_teleop.py"
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


teleop = load_teleop_module()


def quaternion(z_degrees):
    half = math.radians(z_degrees) / 2.0
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


class QuaternionTests(unittest.TestCase):
    def test_distance_uses_shortest_rotation(self):
        identity = quaternion(0.0)
        same_with_opposite_sign = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=-1.0)
        self.assertAlmostEqual(
            teleop.quaternion_distance(identity, same_with_opposite_sign),
            0.0,
        )
        self.assertAlmostEqual(
            teleop.quaternion_distance(identity, quaternion(90.0)),
            math.pi / 2.0,
        )

    def test_relaxation_is_bounded_and_does_not_mutate_reference(self):
        reference = quaternion(0.0)
        relaxed = teleop.relax_orientation(
            reference,
            quaternion(90.0),
            math.radians(3.0),
        )
        self.assertAlmostEqual(
            teleop.quaternion_distance(reference, relaxed),
            math.radians(3.0),
            places=7,
        )
        self.assertEqual(reference.z, 0.0)
        self.assertEqual(reference.w, 1.0)


class TargetLimiterTests(unittest.TestCase):
    def test_enforce_position_preserves_bounded_velocity(self):
        limiter = teleop.CartesianTargetLimiter(0.5, 2.0, 0.04)
        limiter.position = [1.0, 2.0, 3.0]
        limiter.velocity = [0.4, 0.0, 0.0]
        limiter.enforce_position([1.1, 2.1, 3.1])
        self.assertEqual(limiter.position, [1.1, 2.1, 3.1])
        self.assertEqual(limiter.velocity, [0.4, 0.0, 0.0])

    def test_backtracking_scales_velocity_by_accepted_fraction(self):
        limiter = teleop.CartesianTargetLimiter(0.5, 2.0, 0.04)
        limiter.velocity = [0.4, 0.2, 0.0]
        limiter.accept_backtracked_position([0.1, 0.0, 0.0], 0.25)
        self.assertEqual(limiter.position, [0.1, 0.0, 0.0])
        self.assertEqual(limiter.velocity, [0.1, 0.05, 0.0])


class JointContinuityTests(unittest.TestCase):
    def test_joint_lead_is_limited_from_measured_angles(self):
        command = teleop.limited_joint_command(
            {"j0": 0.0, "j1": 1.0},
            {"j0": 0.5, "j1": 0.5},
            0.12,
        )
        self.assertAlmostEqual(command["j0"], 0.12)
        self.assertAlmostEqual(command["j1"], 0.88)

    def test_joint_set_mismatch_is_rejected(self):
        with self.assertRaises(KeyError):
            teleop.maximum_joint_difference({"j0": 0.0}, {"j1": 0.0})


class IkBacktrackingTests(unittest.TestCase):
    class Response:
        RESULT_INVALID = 0

        def __init__(self, valid):
            self.result_type = [1 if valid else 0]
            self.joints = (
                [SimpleNamespace(name=["j0"], position=[0.2])]
                if valid
                else []
            )

    def test_backtracking_keeps_user_seed_and_returns_half_step(self):
        requests = []

        def service(request):
            requests.append(request)
            return self.Response(valid=len(requests) == 2)

        result = teleop.solve_ik_with_backtracking(
            service=service,
            current_position=[0.0, 0.0, 0.0],
            current_orientation=quaternion(0.0),
            desired_position=[1.0, 0.0, 0.0],
            seed_angles={"j0": 0.1},
            attempts=3,
            budget_ms=100.0,
        )

        valid, solution, _code, target, alpha = result
        self.assertTrue(valid)
        self.assertEqual(solution, {"j0": 0.2})
        self.assertAlmostEqual(alpha, 0.5)
        self.assertAlmostEqual(target.pose.position.x, 0.5)
        self.assertEqual(len(requests), 2)
        self.assertTrue(
            all(request.seed_mode == request.SEED_USER for request in requests)
        )
        self.assertTrue(all(request.seed_angles for request in requests))

    def test_elapsed_budget_prevents_another_ik_call(self):
        calls = []

        def slow_invalid_service(_request):
            calls.append(1)
            time.sleep(0.005)
            return self.Response(valid=False)

        valid, _solution, _code, target, alpha = teleop.solve_ik_with_backtracking(
            service=slow_invalid_service,
            current_position=[0.0, 0.0, 0.0],
            current_orientation=quaternion(0.0),
            desired_position=[1.0, 0.0, 0.0],
            seed_angles={"j0": 0.1},
            attempts=3,
            budget_ms=1.0,
        )
        self.assertFalse(valid)
        self.assertIsNone(target)
        self.assertEqual(alpha, 0.0)
        self.assertEqual(len(calls), 1)


class CommandPublisherTests(unittest.TestCase):
    class Limb:
        def __init__(self, fail=False):
            self.fail = fail
            self.commands = []
            self.received = threading.Event()

        def set_joint_positions(self, command, raw=False):
            if self.fail:
                raise RuntimeError("write failed")
            self.commands.append((dict(command), raw))
            self.received.set()

    def test_latest_command_is_published_without_raw_mode(self):
        limb = self.Limb()
        publisher = teleop.ContinuousJointCommandPublisher(limb, rate_hz=100.0)
        publisher.set_command({"j0": 0.25})
        publisher.start()
        try:
            self.assertTrue(limb.received.wait(0.5))
        finally:
            publisher.close()
        self.assertEqual(limb.commands[0], ({"j0": 0.25}, False))
        self.assertIsNone(publisher.error())

    def test_publish_failure_is_exposed_to_main_loop(self):
        limb = self.Limb(fail=True)
        publisher = teleop.ContinuousJointCommandPublisher(limb, rate_hz=100.0)
        publisher.set_command({"j0": 0.25})
        publisher.start()
        try:
            deadline = time.monotonic() + 0.5
            while publisher.error() is None and time.monotonic() < deadline:
                time.sleep(0.005)
        finally:
            publisher.close()
        self.assertIn("write failed", publisher.error())


if __name__ == "__main__":
    unittest.main()
