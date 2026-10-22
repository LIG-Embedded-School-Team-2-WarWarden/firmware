"""Real C++ device processes and optional C++ C2; loopback simulation only."""
import argparse
import copy
import datetime as dt
import json
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "external/warwarden-protocol/python"))
from mfs.icd.v1 import mfs_pb2 as pb

parser = argparse.ArgumentParser()
parser.add_argument("--eff", required=True)
parser.add_argument("--obs")
parser.add_argument("--console")
parser.add_argument("--config")
args, rest = parser.parse_known_args()

def uid():
    return str(uuid.uuid4())

def utc(after=0):
    return (dt.datetime.now(dt.timezone.utc) + dt.timedelta(seconds=after)).isoformat(timespec="milliseconds").replace("+00:00", "Z")

class Device:
    def __init__(self, role="eff"):
        self.id = "201" if role == "eff" else "101"
        self.role, self.boot, self.session, self.control = role, uid(), uid(), uid()
        self.epoch, self.sequence, self.hb = 1, 0, 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(.03)
        self.process = subprocess.Popen([args.eff if role == "eff" else args.obs, "--simulate", "--c2-port", str(self.sock.getsockname()[1])], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.address = None
        try:
            end = time.monotonic() + 5
            while time.monotonic() < end:
                try:
                    raw, addr = self.sock.recvfrom(1200)
                except socket.timeout:
                    continue
                e = pb.Envelope.FromString(raw)
                if e.HasField("register_request"):
                    self.address = addr
                    self.register(e.register_request)
                    self.capabilities = list(e.register_request.capabilities)
                    break
            assert self.address, "device did not register"
            self.until(lambda e: e.HasField("device_status") and e.device_status.motion_state == pb.MOTION_STATE_STOPPED)
            q = pb.InstallationSet(request_id=uid(), device_id=self.id, installation_revision=1, axis_calibration_id="synthetic-" + role + "-zero-v1", mount_rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1])
            self.send(q)
            self.result(q.request_id, "installation_result")
            if role == "eff":
                self.open()
        except BaseException:
            self.close()
            raise

    def register(self, q):
        self.send(pb.RegisterResult(outcome=pb.Result(request_id=q.request_id, result=pb.REQUEST_RESULT_COMPLETED, reported_at=utc(), completed_at=utc()), device_id=self.id, boot_id=q.boot_id, connection_session_id=self.session, c2_boot_id=self.boot))

    def send(self, body):
        e = pb.Envelope(version_major=1, sender_id="C2", recipient_id=self.id, connection_session_id=self.session, message_id=uid(), sent_at=utc())
        name = next(f.name for f in pb.Envelope.DESCRIPTOR.oneofs_by_name["body"].fields if f.message_type.full_name == body.DESCRIPTOR.full_name)
        getattr(e, name).CopyFrom(body)
        self.sock.sendto(e.SerializeToString(), self.address)

    def until(self, predicate, timeout=5):
        end, last = time.monotonic() + timeout, 0
        while time.monotonic() < end:
            if time.monotonic() - last > .2:
                last = time.monotonic()
                self.hb += 1
                self.send(pb.Heartbeat(boot_id=self.boot, sequence=self.hb))
            try:
                raw, _ = self.sock.recvfrom(1200)
            except socket.timeout:
                continue
            e = pb.Envelope.FromString(raw)
            if predicate(e):
                return e
        raise AssertionError("device response timed out")

    def result(self, request, body="laser_result"):
        return getattr(self.until(lambda e: e.HasField(body) and getattr(e, body).outcome.request_id == request and getattr(e, body).outcome.result != pb.REQUEST_RESULT_ACCEPTED), body)

    def open(self):
        q = pb.ControlOpen(request_id=uid(), control_session_id=self.control, control_epoch=self.epoch, installation_revision=1, mode=pb.CONTROL_MODE_MANUAL)
        self.send(q)
        assert self.result(q.request_id, "control_result").outcome.result == pb.REQUEST_RESULT_COMPLETED

    def command(self, operation=pb.LaserCommand.OPERATION_ON, duration=1000):
        self.sequence += 1
        q = pb.LaserCommand(request_id=uid(), control_session_id=self.control, control_epoch=self.epoch, command_sequence=self.sequence, operation=operation, issued_at=utc(), effector_installation_revision=1)
        if operation == pb.LaserCommand.OPERATION_ON:
            q.duration_ms, q.expires_at = duration, utc(1)
        return q

    def execute(self, q):
        self.send(q)
        return self.result(q.request_id)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
        self.process.wait(timeout=5)
        self.process.stderr.close()
        self.sock.close()

class LaserWire(unittest.TestCase):
    def setUp(self):
        self.device = Device()
        self.addCleanup(self.device.close)

    def test_duplicate_does_not_extend_timer_and_conflict_is_rejected(self):
        d = self.device
        self.assertIn(pb.CAPABILITY_LASER_CONTROL, d.capabilities)
        q = d.command(duration=600)
        r = d.execute(q)
        self.assertEqual(r.actual_laser_state, pb.LASER_STATE_ON)
        self.assertTrue(r.HasField("state_observed_at"))
        self.assertEqual(d.execute(d.command()).outcome.reason_code, pb.REASON_CODE_BUSY)
        conflict = copy.deepcopy(q)
        conflict.duration_ms = 700
        self.assertEqual(d.execute(conflict).outcome.reason_code, pb.REASON_CODE_ID_CONFLICT)
        time.sleep(.35)
        self.assertEqual(d.execute(q).outcome.result, pb.REQUEST_RESULT_COMPLETED)
        d.until(lambda e: e.HasField("laser_status") and e.laser_status.laser_state == pb.LASER_STATE_OFF and e.laser_status.last_accepted_sequence == 1, timeout=.5)
        self.assertEqual(d.execute(d.command(pb.LaserCommand.OPERATION_GET_STATUS)).actual_laser_state, pb.LASER_STATE_OFF)

    def test_invalid_duration_expiry_session_revision_and_sequence(self):
        d = self.device
        for mutate, reason in [
            (lambda q: q.ClearField("duration_ms"), pb.REASON_CODE_INVALID_ARGUMENT),
            (lambda q: q.ClearField("expires_at"), pb.REASON_CODE_INVALID_ARGUMENT),
            (lambda q: setattr(q, "duration_ms", 0), pb.REASON_CODE_INVALID_ARGUMENT),
            (lambda q: setattr(q, "duration_ms", 5001), pb.REASON_CODE_OUT_OF_RANGE),
            (lambda q: setattr(q, "expires_at", utc(-1)), pb.REASON_CODE_EXPIRED_COMMAND),
            (lambda q: setattr(q, "control_session_id", uid()), pb.REASON_CODE_STALE_COMMAND),
            (lambda q: setattr(q, "effector_installation_revision", 2), pb.REASON_CODE_INSTALLATION_MISMATCH),
            (lambda q: setattr(q, "issued_at", "bad"), pb.REASON_CODE_INVALID_ARGUMENT),
        ]:
            q = d.command()
            mutate(q)
            self.assertEqual(d.execute(q).outcome.reason_code, reason)
        q = d.command(pb.LaserCommand.OPERATION_OFF)
        self.assertEqual(d.execute(q).actual_laser_state, pb.LASER_STATE_OFF)
        old = d.command()
        old.command_sequence = q.command_sequence
        self.assertEqual(d.execute(old).outcome.reason_code, pb.REASON_CODE_STALE_COMMAND)

    def test_stop_and_drain_disable_output_and_close_gate(self):
        d = self.device
        for operation in (pb.PointingCommand.OPERATION_STOP, pb.PointingCommand.OPERATION_DRAIN):
            self.assertEqual(d.execute(d.command()).actual_laser_state, pb.LASER_STATE_ON)
            q = pb.PointingCommand(request_id=uid(), control_session_id=d.control, control_epoch=d.epoch, command_sequence=1, operation=operation, issued_at=utc())
            d.send(q)
            self.assertEqual(d.result(q.request_id, "pointing_result").outcome.result, pb.REQUEST_RESULT_COMPLETED)
            self.assertEqual(d.execute(d.command(pb.LaserCommand.OPERATION_GET_STATUS)).actual_laser_state, pb.LASER_STATE_OFF)
            self.assertEqual(d.execute(d.command()).outcome.reason_code, pb.REASON_CODE_BUSY)
            self.assertEqual(d.execute(d.command(pb.LaserCommand.OPERATION_OFF)).outcome.result, pb.REQUEST_RESULT_COMPLETED)
            d.epoch += 1
            d.control = uid()
            d.sequence = 0
            d.open()

    def test_control_replacement_disables_output_and_resets_sequence(self):
        d = self.device
        self.assertEqual(d.execute(d.command()).actual_laser_state, pb.LASER_STATE_ON)
        d.epoch += 1
        d.control, d.sequence = uid(), 0
        d.open()
        self.assertEqual(d.execute(d.command(pb.LaserCommand.OPERATION_GET_STATUS)).actual_laser_state, pb.LASER_STATE_OFF)

    def test_connection_loss_disables_output_before_reregistration(self):
        d = self.device
        self.assertEqual(d.execute(d.command(duration=5000)).actual_laser_state, pb.LASER_STATE_ON)
        time.sleep(2.3)  # No C2 heartbeat; output must be off before its five-second timer.
        q = d.until(lambda e: e.HasField("register_request")).register_request
        d.session, d.boot, d.hb = uid(), uid(), 0
        d.register(q)
        e = d.until(lambda e: e.HasField("laser_status") and e.laser_status.control_epoch == 0)
        self.assertEqual(e.laser_status.laser_state, pb.LASER_STATE_OFF)

    @unittest.skipUnless(args.obs, "OBS executable not supplied")
    def test_obs_does_not_advertise_or_accept_laser(self):
        d = Device("obs")
        self.addCleanup(d.close)
        self.assertNotIn(pb.CAPABILITY_LASER_CONTROL, d.capabilities)
        self.assertEqual(d.execute(d.command()).outcome.reason_code, pb.REASON_CODE_UNSUPPORTED_TYPE)

@unittest.skipUnless(args.console and args.config, "native C2 executable/config not supplied")
class NativeConsole(unittest.TestCase):
    def test_operator_laser_commands_and_stop(self):
        console = subprocess.Popen([args.console, "--config", args.config, "--asset-port", "0"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8")
        lines = queue.Queue()
        def read():
            for line in console.stdout:
                lines.put(json.loads(line))
        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        eff = None
        def command(text):
            console.stdin.write(text + "\n")
            console.stdin.flush()
            return lines.get(timeout=5)
        def status(predicate):
            end = time.monotonic() + 6
            while time.monotonic() < end:
                value = command("status")
                if predicate(value):
                    return value
                time.sleep(.03)
            self.fail("native C2 status timed out: " + repr(value))
        try:
            ready = lines.get(timeout=5)
            eff = subprocess.Popen([args.eff, "--simulate", "--c2-port", str(ready["port"])], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            status(lambda s: s["devices"].get("201", {}).get("installed"))
            self.assertEqual(command("point 201 0 0"), {"accepted": "point"})
            status(lambda s: s["controls"].get("201", {}).get("reason") == "POINT_COMPLETED")
            self.assertEqual(command("laser on 201 500"), {"accepted": "laser"})
            status(lambda s: s["lasers"]["201"]["state"] == "LASER_STATE_ON")
            status(lambda s: s["lasers"]["201"]["state"] == "LASER_STATE_OFF")
            command("laser status 201")
            status(lambda s: s["lasers"]["201"]["result"] == "REQUEST_RESULT_COMPLETED")
            command("laser on 201 5000")
            status(lambda s: s["lasers"]["201"]["state"] == "LASER_STATE_ON")
            command("stop 201")
            status(lambda s: s["controls"]["201"]["state"] == "STOPPED" and s["lasers"]["201"]["state"] == "LASER_STATE_OFF")
            command("quit")
            console.wait(timeout=5)
            self.assertEqual(console.returncode, 0)
        finally:
            if console.poll() is None:
                console.terminate()
                console.wait(timeout=5)
            if eff:
                eff.terminate()
                eff.wait(timeout=5)
            reader.join(timeout=1)
            console.stdin.close()
            console.stdout.close()

if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]] + rest, verbosity=2)
