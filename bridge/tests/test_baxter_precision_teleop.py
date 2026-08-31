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
    def test_copy_quaternion_does_not_depend_on_source_mutability(self):
        source = quaternion(30.0)
        copied = teleop.copy_quaternion(source)
        self.assertIsInstance(copied, teleop.QuaternionValue)
        self.assertIsNot(copied, source)
        self.assertAlmostEqual(copied.z, source.z)
        self.assertAlmostEqual(copied.w, source.w)

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


class ControllerInputTests(unittest.TestCase):
    def test_trigger_is_clamped_and_invalid_values_are_safe(self):
        self.assertEqual(
            teleop.controller_trigger({"left": {"trigger": 2.0}}, "left"),
            1.0,
        )
        self.assertEqual(
            teleop.controller_trigger({"left": {"trigger": -1.0}}, "left"),
            0.0,
        )
        self.assertEqual(
            teleop.controller_trigger({"left": {"trigger": "bad"}}, "left"),
            0.0,
        )
        self.assertEqual(
            teleop.controller_trigger({"left": {"trigger": float("nan")}}, "left"),
            0.0,
        )


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


class GripperPublisherTests(unittest.TestCase):
    class Gripper:
        name = "left_gripper"

        def __init__(self, gripper_type="electric", calibrated=True):
            self.gripper_type = gripper_type
            self.is_calibrated = calibrated
            self.has_error = False
            self.positions = []
            self.suction_commands = []
            self.received = threading.Event()

        def type(self):
            return self.gripper_type

        def calibrated(self):
            return self.is_calibrated

        def error(self):
            return self.has_error

        def command_position(self, position, block=False):
            self.positions.append((position, block))
            self.received.set()
            return True

        def close(self, block=False):
            self.suction_commands.append(("close", block))
            return True

        def open(self, block=False):
            self.suction_commands.append(("open", block))
            return True

    def publisher(self, gripper):
        return teleop.LatestGripperCommandPublisher(
            gripper=gripper,
            rate_hz=100.0,
            electric_deadband=2.0,
            suction_press=0.6,
            suction_release=0.4,
        )

    def test_electric_trigger_is_inverted_and_non_blocking(self):
        gripper = self.Gripper()
        publisher = self.publisher(gripper)
        publisher.set_trigger(0.75)
        publisher.start()
        try:
            self.assertTrue(gripper.received.wait(0.5))
        finally:
            publisher.close()
        self.assertEqual(gripper.positions[0], (25.0, False))
        self.assertIsNone(publisher.error())

    def test_electric_deadband_suppresses_nearby_commands(self):
        gripper = self.Gripper()
        publisher = self.publisher(gripper)
        publisher._command_electric(0.50)
        publisher._command_electric(0.51)
        publisher._command_electric(0.55)
        self.assertEqual(len(gripper.positions), 2)
        self.assertAlmostEqual(gripper.positions[0][0], 50.0)
        self.assertAlmostEqual(gripper.positions[1][0], 45.0)
        self.assertFalse(gripper.positions[0][1])
        self.assertFalse(gripper.positions[1][1])

    def test_suction_uses_hysteresis(self):
        gripper = self.Gripper(gripper_type="suction")
        publisher = self.publisher(gripper)
        for trigger in (0.0, 0.5, 0.7, 0.5, 0.2):
            publisher._command_suction(trigger)
        self.assertEqual(
            gripper.suction_commands,
            [("open", False), ("close", False), ("open", False)],
        )

    def test_uncalibrated_electric_gripper_is_rejected_without_calibrating(self):
        gripper = self.Gripper(calibrated=False)
        with self.assertRaisesRegex(ValueError, "not calibrated"):
            self.publisher(gripper)

    def test_runtime_gripper_error_stops_only_gripper_worker(self):
        gripper = self.Gripper()
        publisher = self.publisher(gripper)
        gripper.has_error = True
        publisher.set_trigger(0.5)
        publisher.start()
        try:
            deadline = time.monotonic() + 0.5
            while publisher.error() is None and time.monotonic() < deadline:
                time.sleep(0.002)
        finally:
            publisher.close()
        self.assertIn("error state", publisher.error())
        self.assertEqual(gripper.positions, [])


class AsyncIkWorkerTests(unittest.TestCase):
    class Response:
        RESULT_INVALID = 0

        def __init__(self):
            self.result_type = [1]
            self.joints = [SimpleNamespace(name=["j0"], position=[0.2])]

    def wait_for_result(self, worker, timeout=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = worker.poll_result()
            if result is not None:
                return result
            time.sleep(0.002)
        self.fail("timed out waiting for IK result")

    def submit(self, worker, session_id, sequence, x):
        return worker.submit(
            session_id=session_id,
            sequence=sequence,
            current_position=[0.0, 0.0, 0.0],
            current_orientation=quaternion(0.0),
            desired_position=[x, 0.0, 0.0],
            seed_angles={"j0": 0.1},
            attempts=1,
            budget_ms=100.0,
        )

    def test_worker_is_single_flight_and_drops_stale_pending_seed(self):
        entered = threading.Event()
        release = threading.Event()
        requested_x = []

        def service(request):
            requested_x.append(request.pose_stamp[0].pose.position.x)
            entered.set()
            release.wait(0.5)
            return self.Response()

        worker = teleop.LatestOnlyIkWorker(service)
        worker.start()
        try:
            first_id = self.submit(worker, session_id=1, sequence=10, x=0.1)
            self.assertTrue(entered.wait(0.5))
            self.submit(worker, session_id=1, sequence=11, x=0.2)
            self.submit(worker, session_id=1, sequence=12, x=0.3)
            self.assertEqual(worker.snapshot().replaced_pending, 1)

            release.set()
            result = self.wait_for_result(worker)
            self.assertEqual(result.request.request_id, first_id)
            self.assertEqual(requested_x, [0.1])

            # poll_result discarded the pre-result pending request.  Only a
            # freshly submitted target may start with the refreshed seed.
            time.sleep(0.02)
            self.assertEqual(requested_x, [0.1])
            self.submit(worker, session_id=1, sequence=13, x=0.4)
            second = self.wait_for_result(worker)
            self.assertEqual(second.request.sequence, 13)
            self.assertEqual(requested_x, [0.1, 0.4])
        finally:
            worker.close()


    def test_cancel_pending_cannot_leak_old_session_result(self):
        entered = threading.Event()
        release = threading.Event()

        def service(_request):
            entered.set()
            release.wait(0.5)
            return self.Response()

        worker = teleop.LatestOnlyIkWorker(service)
        worker.start()
        try:
            self.submit(worker, session_id=4, sequence=20, x=0.1)
            self.assertTrue(entered.wait(0.5))
            worker.cancel_pending()
            release.set()
            result = self.wait_for_result(worker)
            reason = teleop.ik_result_rejection_reason(
                result,
                active_session_id=5,
                now_monotonic=time.monotonic(),
                maximum_age=1.0,
            )
            self.assertIn("inactive Grip session", reason)
        finally:
            worker.close()


    def test_old_result_is_rejected_by_age(self):
        request = teleop.IkWorkItem(
            request_id=1,
            session_id=2,
            sequence=3,
            submitted_monotonic=10.0,
            current_position=[0.0, 0.0, 0.0],
            current_orientation=teleop.QuaternionValue(0.0, 0.0, 0.0, 1.0),
            desired_position=[0.1, 0.0, 0.0],
            seed_angles={"j0": 0.0},
            attempts=1,
            budget_ms=10.0,
        )
        result = teleop.IkWorkResult(
            request=request,
            started_monotonic=10.0,
            completed_monotonic=10.2,
            valid=True,
            solution={"j0": 0.1},
            result_code=1,
            accepted_target=None,
            accepted_alpha=1.0,
            error=None,
        )
        reason = teleop.ik_result_rejection_reason(
            result,
            active_session_id=2,
            now_monotonic=11.0,
            maximum_age=0.25,
        )
        self.assertIn("stale IK result", reason)

    def test_many_submissions_keep_only_one_pending_request(self):
        worker = teleop.LatestOnlyIkWorker(lambda _request: self.Response())
        try:
            started = time.monotonic()
            last_id = None
            for sequence in range(1000):
                last_id = self.submit(
                    worker,
                    session_id=7,
                    sequence=sequence,
                    x=sequence / 1000.0,
                )
            elapsed = time.monotonic() - started
            snapshot = worker.snapshot()
            self.assertEqual(snapshot.pending_request_id, last_id)
            self.assertEqual(snapshot.replaced_pending, 999)
            self.assertLess(elapsed, 0.25)
        finally:
            worker.close()

    def test_service_exception_is_returned_without_killing_worker(self):
        calls = []

        def failing_service(_request):
            calls.append(1)
            raise RuntimeError("service unavailable")

        worker = teleop.LatestOnlyIkWorker(failing_service)
        worker.start()
        try:
            self.submit(worker, session_id=1, sequence=1, x=0.1)
            first = self.wait_for_result(worker)
            self.assertFalse(first.valid)
            self.assertIn("service unavailable", first.error)

            self.submit(worker, session_id=1, sequence=2, x=0.2)
            second = self.wait_for_result(worker)
            self.assertEqual(second.request.sequence, 2)
            self.assertEqual(len(calls), 2)
        finally:
            worker.close()


class PyKdlIkBackendTests(unittest.TestCase):
    def request(self, desired=None, seed=None, attempts=3):
        return teleop.IkWorkItem(
            request_id=1,
            session_id=1,
            sequence=1,
            submitted_monotonic=time.monotonic(),
            current_position=[0.0, 0.0, 0.0],
            current_orientation=teleop.QuaternionValue(
                0.0, 0.0, 0.0, 1.0
            ),
            desired_position=desired or [0.2, 0.0, 0.0],
            seed_angles=(
                {"j0": 0.1, "j1": -0.1} if seed is None else seed
            ),
            attempts=attempts,
            budget_ms=100.0,
        )

    def test_local_backend_uses_ordered_seed_and_full_target(self):
        calls = []

        class Kinematics:
            def inverse_kinematics(self, position, orientation, seed):
                calls.append((position, orientation, seed))
                return [0.2, -0.2]

        backend = teleop.PyKdlIkBackend(Kinematics(), ["j0", "j1"])
        valid, solution, code, target, alpha = backend.solve(self.request())

        self.assertTrue(valid)
        self.assertEqual(solution, {"j0": 0.2, "j1": -0.2})
        self.assertEqual(code, 1)
        self.assertEqual(alpha, 1.0)
        self.assertAlmostEqual(target.pose.position.x, 0.2)
        self.assertEqual(calls[0][1], [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(calls[0][2], [0.1, -0.1])

    def test_local_backend_backtracks_without_changing_seed(self):
        calls = []

        class Kinematics:
            def inverse_kinematics(self, position, _orientation, seed):
                calls.append((list(position), list(seed)))
                return None if len(calls) == 1 else [0.15, -0.15]

        backend = teleop.PyKdlIkBackend(Kinematics(), ["j0", "j1"])
        valid, _solution, _code, target, alpha = backend.solve(self.request())

        self.assertTrue(valid)
        self.assertEqual(alpha, 0.5)
        self.assertAlmostEqual(target.pose.position.x, 0.1)
        self.assertEqual(calls[0][1], calls[1][1])

    def test_local_backend_rejects_wrong_seed_joint_set(self):
        backend = teleop.PyKdlIkBackend(object(), ["j0", "j1"])
        with self.assertRaises(KeyError):
            backend.solve(self.request(seed={"j0": 0.0}))


if __name__ == "__main__":
    unittest.main()
