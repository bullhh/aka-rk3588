"""Exercise the vision+FT232 loopback launcher contract without board devices."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMPLETE = """[VISION_USB_CI] DEVICE transport=usb ft232=0403:6001 serial=SN0001 bus=1 addr=4 iface=0 ep_in=0x81 ep_out=0x02 tty=- camera_uvc_index=0
[VISION_USB_CI] PERF_BEGIN windows=2 duration_s=10 min_fps=28.00 cadence_hz=20 min_tx=160
[VISION_USB_CI] PERF_WINDOW index=1/2 elapsed_s=10.02 processed=300 effective_fps=29.94
[VISION_USB_CI] LOOPBACK_WINDOW index=1/2 attempts=201 tx=200 errors=0 avg_ms=2.100 max_ms=5.300
[VISION_USB_CI] PERF_WINDOW index=2/2 elapsed_s=10.01 processed=300 effective_fps=29.97
[VISION_USB_CI] LOOPBACK_WINDOW index=2/2 attempts=200 tx=199 errors=0 avg_ms=2.200 max_ms=6.100
[VISION_USB_CI] PERF_SUMMARY windows=2 elapsed_s=20.03 processed=600 effective_fps=29.96 min_fps=28.00
[VISION_USB_CI] LOOPBACK_SUMMARY attempts=401 tx=399 errors=0 min_tx=320
[VISION_USB_CI] UVC_PAUSE_RESUME=PASS fresh_frame=1
[VISION_USB_CI] APPLICATION_PASS windows=2 min_fps=28.00 processed=600 elapsed_s=20.03 effective_fps=29.96 loopback_tx=399 loopback_errors=0 loopback_min_tx=320 ft232_serial=SN0001 ft232_bus=1 ft232_addr=4 camera_uvc_index=0 pause_resume=1
"""


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
