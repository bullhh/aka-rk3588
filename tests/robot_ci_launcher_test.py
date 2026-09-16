"""Exercise the real CI launcher acceptance contract without board devices."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMPLETE = """[ROBOT_CI] PERF_BEGIN windows=2 duration_s=10 min_fps=28.00
[ROBOT_CI] PERF_WINDOW index=1/2 elapsed_s=10.00 processed=300 effective_fps=30.00
[ROBOT_CI] PERF_WINDOW index=2/2 elapsed_s=10.00 processed=300 effective_fps=30.00
[ROBOT_CI] PERF_SUMMARY windows=2 elapsed_s=20.00 processed=600 effective_fps=30.00
[ROBOT_CI] SAFE_POSE=PASS pose=carry source=flow
[ROBOT_CI] ATTEMPT_PASS flow=1 perf_windows=2 ball_seen=0 ball_drive=1 bucket_drive=1 safe_stop=1
"""


class RobotCiLauncherTest(unittest.TestCase):
    def launch(self, output=COMPLETE, status=0, minimum="28"):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build").mkdir()
            (root / "models").mkdir()
            (root / "bin").mkdir()
            for name, text in [("lsusb", "echo attached"), ("sleep", ":")]:
                helper = root / "bin" / name
                helper.write_text("#!/bin/sh\n" + text + "\n")
                helper.chmod(0o755)
            (root / "models/tennis.rknn").touch()
            script = root / "run_robot_ci_once.sh"
            script.write_bytes((ROOT / script.name).read_bytes())
            binary = root / "build/tennis"
            binary.write_text('#!/bin/sh\n[ "$1" = test-new-arm ] && exit 0\ncat <<\'OUTPUT\'\n' + output +
                              "OUTPUT\nexit " + str(status) + "\n")
            binary.chmod(0o755)
            env = dict(os.environ, FEETECH_DEV="auto", PATH=str(root / "bin") + os.pathsep + os.environ["PATH"])
            env.pop("MODEL_PATH", None)
            return subprocess.run(["sh", str(script), minimum], env=env,
                                  capture_output=True, text=True, timeout=5)

    def test_complete_flow_and_exit_are_both_required(self):
        passed = self.launch()
        self.assertEqual(passed.returncode, 0, passed.stdout + passed.stderr)
        self.assertIn("RESULT=PASS attempts=1", passed.stdout)
        for output, status in [
            ("", 0), (COMPLETE, 7),
            (COMPLETE.replace("effective_fps=30.00", "effective_fps=27.00"), 0),
            (COMPLETE.replace("min_fps=28.00", "min_fps=15.00"), 0),
            (COMPLETE.replace("index=2/2", "index=1/2"), 0),
            (COMPLETE.replace("safe_stop=1", "safe_stop=0"), 0),
            (COMPLETE.replace("ball_drive=1", "ball_drive=0"), 0),
            (COMPLETE + COMPLETE, 0),
        ]:
            with self.subTest(output=output, status=status):
                result = self.launch(output, status)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertNotIn("RESULT=PASS", result.stdout)

    def test_invalid_threshold_never_runs_the_application(self):
        for minimum in ("NaN", "0", "-1", "1001", "28 trailing"):
            with self.subTest(minimum=minimum):
                result = self.launch(minimum=minimum)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("PERF_BEGIN", result.stdout)


if __name__ == "__main__":
    unittest.main()
