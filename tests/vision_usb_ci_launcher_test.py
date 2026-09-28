"""Exercise the vision+FT232 loopback launcher contract without board devices."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMPLETE = """[VISION_USB_CI] DEVICE transport=usb ft232=0403:6001 serial=SN0001 bus=1 addr=4 iface=0 ep_in=0x81 ep_out=0x02 tty=- camera_uvc_index=0
[VISION_USB_CI] PERF_BEGIN windows=2 duration_s=10 min_fps=28.00 cadence_hz=20 min_tx=160
[VISION_USB_CI] PERF_WINDOW index=1/2 elapsed_s=10.02 processed=300 effective_fps=29.94 jpeg_errors=0
[VISION_USB_CI] LOOPBACK_WINDOW index=1/2 attempts=201 tx=200 errors=0 avg_ms=2.100 max_ms=5.300
[VISION_USB_CI] PERF_WINDOW index=2/2 elapsed_s=10.01 processed=300 effective_fps=29.97 jpeg_errors=0
[VISION_USB_CI] LOOPBACK_WINDOW index=2/2 attempts=200 tx=199 errors=0 avg_ms=2.200 max_ms=6.100
[VISION_USB_CI] PERF_SUMMARY windows=2 elapsed_s=20.03 processed=600 effective_fps=29.96 min_fps=28.00 jpeg_errors=0
[VISION_USB_CI] LOOPBACK_SUMMARY attempts=401 tx=399 errors=0 min_tx=320
[VISION_USB_CI] UVC_PAUSE_RESUME=PASS fresh_frame=1
[VISION_USB_CI] APPLICATION_PASS windows=2 min_fps=28.00 processed=600 elapsed_s=20.03 effective_fps=29.96 loopback_tx=399 loopback_errors=0 loopback_min_tx=320 jpeg_errors=0 ft232_serial=SN0001 ft232_bus=1 ft232_addr=4 camera_uvc_index=0 pause_resume=1
"""

# One tolerated single-frame decode error in window 1, kept consistent across
# the window line, the summary, and the final verdict. One error is inside the
# per-window tolerance of 3.
TOLERATED = (COMPLETE
             .replace("effective_fps=29.94 jpeg_errors=0",
                      "effective_fps=29.94 jpeg_errors=1")
             .replace("min_fps=28.00 jpeg_errors=0",
                      "min_fps=28.00 jpeg_errors=1")
             .replace("jpeg_errors=0 ft232_serial", "jpeg_errors=1 ft232_serial"))

# Exactly the per-window budget in window 1 plus two in window 2 (5 total).
# Exercises the per-window boundary and a nonzero aggregate.
TOLERATED_MAX = (COMPLETE
                 .replace("effective_fps=29.94 jpeg_errors=0",
                          "effective_fps=29.94 jpeg_errors=3")
                 .replace("effective_fps=29.97 jpeg_errors=0",
                          "effective_fps=29.97 jpeg_errors=2")
                 .replace("jpeg_errors=0", "jpeg_errors=5"))


class VisionUsbCiLauncherTest(unittest.TestCase):
    def launch(self, output=COMPLETE, status=0, minimum="28"):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build").mkdir()
            (root / "models").mkdir()
            (root / "models/tennis.rknn").touch()
            script = root / "run_vision_usb_ci_once.sh"
            script.write_bytes((ROOT / script.name).read_bytes())
            binary = root / "build/tennis"
            binary.write_text('#!/bin/sh\ncat <<\'OUTPUT\'\n' + output +
                              "OUTPUT\nexit " + str(status) + "\n")
            binary.chmod(0o755)
            env = dict(os.environ)
            env.pop("MODEL_PATH", None)
            env.pop("FT232_SERIAL", None)
            env.pop("FTDI_TRANSPORT", None)
            return subprocess.run(["sh", str(script), minimum], env=env,
                                  capture_output=True, text=True, timeout=10)

    def test_complete_flow_and_exit_are_both_required(self):
        passed = self.launch()
        self.assertEqual(passed.returncode, 0, passed.stdout + passed.stderr)
        self.assertIn("RESULT=PASS attempts=1", passed.stdout)
        for output, status in [
            ("", 0), (COMPLETE, 7),
            (COMPLETE.replace("effective_fps=29.96", "effective_fps=27.00"), 0),
            (COMPLETE.replace("min_fps=28.00", "min_fps=15.00"), 0),
            (COMPLETE.replace("APPLICATION_PASS", "ATTEMPT_PASS"), 0),
            (COMPLETE.replace("windows=2", "windows=1"), 0),
            (COMPLETE.replace("pause_resume=1", "pause_resume=0"), 0),
            (COMPLETE.replace("loopback_errors=0", "loopback_errors=3"), 0),
            (COMPLETE.replace("loopback_tx=399", "loopback_tx=300"), 0),
            (COMPLETE.replace("loopback_min_tx=320", "loopback_min_tx=600"), 0),
            (COMPLETE.replace("loopback_min_tx=320", "loopback_min_tx=100"), 0),
            (COMPLETE.replace("tx=200", "tx=100"), 0),
            (COMPLETE.replace("tx=199", "tx=159"), 0),
            (COMPLETE.replace("transport=usb", "transport=serial"), 0),
            (COMPLETE.replace("[VISION_USB_CI] DEVICE", "[VISION_USB_CI] NODE"), 0),
            (COMPLETE.replace("elapsed_s=20.03", "elapsed_s=19.99"), 0),
            (COMPLETE.replace("processed=600", "processed=0"), 0),
            (COMPLETE.replace("processed=300", "processed=0"), 0),
            (COMPLETE.replace("elapsed_s=10.02", "elapsed_s=9.50"), 0),
            (COMPLETE[:COMPLETE.index("[VISION_USB_CI] APPLICATION_PASS")], 0),
            (COMPLETE + COMPLETE, 0),
            (COMPLETE.replace("PERF_WINDOW index=2/2", "PERF_WINDOW index=1/2"), 0),
            # An old binary that never reports jpeg_errors can never pass.
            (COMPLETE.replace(" jpeg_errors=0", ""), 0),
            # A window above the per-frame tolerance of 3 must fail even when the
            # aggregate is still inside the 0..6 total budget.
            (COMPLETE.replace("effective_fps=29.94 jpeg_errors=0",
                              "effective_fps=29.94 jpeg_errors=4"), 0),
            (COMPLETE.replace("effective_fps=29.94 jpeg_errors=0",
                              "effective_fps=29.94 jpeg_errors=7"), 0),
            (COMPLETE.replace("jpeg_errors=0 ft232_serial",
                              "jpeg_errors=7 ft232_serial"), 0),
            (COMPLETE.replace("min_fps=28.00 jpeg_errors=0",
                              "min_fps=28.00 jpeg_errors=6"), 0),
            # Interior and final totals must agree with the two window lines.
            (TOLERATED_MAX.replace("min_fps=28.00 jpeg_errors=5",
                                   "min_fps=28.00 jpeg_errors=1"), 0),
            (TOLERATED_MAX.replace("jpeg_errors=5 ft232_serial",
                                   "jpeg_errors=4 ft232_serial"), 0),
        ]:
            with self.subTest(output=output[:60], status=status):
                result = self.launch(output, status)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertNotIn("RESULT=PASS", result.stdout)

    def test_only_the_final_application_verdict_establishes_success(self):
        verdict = COMPLETE[COMPLETE.index("[VISION_USB_CI] APPLICATION_PASS"):]
        # The final marker alone cannot pass without the two window reports.
        failed = self.launch(verdict)
        self.assertNotEqual(failed.returncode, 0, failed.stdout)
        self.assertNotIn("RESULT=PASS", failed.stdout)

        diagnostics = COMPLETE[:COMPLETE.index("[VISION_USB_CI] APPLICATION_PASS")]
        failed = self.launch(diagnostics + "Different error wording\n", 1)
        self.assertNotEqual(failed.returncode, 0, failed.stdout)
        self.assertNotIn("RESULT=PASS", failed.stdout)

    def test_tolerated_decode_errors_within_budget_still_pass(self):
        for output in (TOLERATED, TOLERATED_MAX):
            with self.subTest(output=output[:60]):
                passed = self.launch(output)
                self.assertEqual(passed.returncode, 0,
                                 passed.stdout + passed.stderr)
                self.assertIn("RESULT=PASS attempts=1", passed.stdout)

    def test_invalid_threshold_never_runs_the_application(self):
        for minimum in ("NaN", "0", "-1", "1001", "28 trailing"):
            with self.subTest(minimum=minimum):
                result = self.launch(minimum=minimum)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("PERF_BEGIN", result.stdout)
                self.assertNotIn("RESULT=PASS", result.stdout)

    def test_no_legacy_robot_marker_is_emitted(self):
        passed = self.launch()
        self.assertEqual(passed.returncode, 0, passed.stdout + passed.stderr)
        self.assertNotIn("[ROBOT_CI]", passed.stdout)


if __name__ == "__main__":
    unittest.main()
