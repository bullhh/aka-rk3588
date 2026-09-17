"""Exercise production wheel checks through a Feetech serial peer on a PTY."""
import os
import pty
import select
import subprocess
import sys
import threading
import tempfile
import unittest

DRIVER = sys.argv.pop(1)


class WheelFeedbackTest(unittest.TestCase):
    def run_peer(self, fault):
        master, slave = pty.openpty()
        finished = threading.Event()
        errors = []
        goals = {7: 0, 8: 0, 9: 0}
        saw_motion = False

        def peer():
            nonlocal saw_motion
            pending = bytearray()
            try:
                while not finished.is_set():
                    if not select.select([master], [], [], 0.05)[0]:
                        continue
                    pending.extend(os.read(master, 4096))
                    while len(pending) >= 4:
                        self.assertEqual(pending[:2], b"\xff\xff")
                        size = pending[3] + 4
                        if len(pending) < size:
                            break
                        packet = bytes(pending[:size])
                        del pending[:size]
                        self.assertEqual(sum(packet[2:]) & 255, 255)
                        servo, instruction = packet[2], packet[4]
                        params = packet[5:-1]
                        if instruction == 0x83:
                            self.assertEqual((servo, params[0], params[1]), (254, 46, 2))
                            for i in range(2, len(params), 3):
                                raw = params[i + 1] | params[i + 2] << 8
                                goals[params[i]] = -(raw & 32767) if raw & 32768 else raw
                            saw_motion |= any(goals.values())
                            continue  # Broadcast writes have no status response.
                        self.assertEqual((instruction, params), (2, bytes([58, 2])))
                        if fault == "read-error":
                            continue
                        velocity = goals[servo]
                        if fault == "one-stalled" and servo == 8:
                            velocity = 0
                        elif fault == "all-stalled":
                            velocity = 0
                        elif fault == "wrong-direction" and servo == 9:
                            velocity = -velocity
                        elif fault == "stop-failure" and saw_motion and not velocity:
                            velocity = 100
                        raw = abs(velocity) | (32768 if velocity < 0 else 0)
                        body = bytes([servo, 4, 0, raw & 255, raw >> 8])
                        os.write(master, b"\xff\xff" + body + bytes([(~sum(body)) & 255]))
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=peer)
        worker.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                port = os.path.join(directory, "port")
                os.symlink(os.ttyname(slave), port)
                result = subprocess.run([DRIVER, port, fault],
                                        capture_output=True, text=True, timeout=12)
        finally:
            finished.set()
            worker.join(timeout=1)
            os.close(master)
            os.close(slave)
        if errors:
            raise errors[0]
        self.assertFalse(worker.is_alive())
        return result

    def test_all_wheels_must_respond_and_stop(self):
        for fault in ("none", "one-stalled", "all-stalled", "wrong-direction",
                      "stop-failure", "read-error", "command-error"):
            with self.subTest(fault=fault):
                result = self.run_peer(fault)
                self.assertEqual(result.returncode, 0 if fault == "none" else 1,
                                 result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
