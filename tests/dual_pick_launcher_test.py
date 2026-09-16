"""Run the real launchers with a fake executable and isolated device paths."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DualPickLauncherTests(unittest.TestCase):
    def launch(self, linux, ci=True, status=0, args=(), complete=True,
               fps='30', reported_threshold=None, windows=2, duration='62000', perf='pass',
               control=True, control_session='17', control_status='ok'):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('bin', 'models', 'config'):
                (root / name).mkdir()
            for name in ('models/tennis.rknn', 'config/lekiwi_calibration.json',
                         'config/lekiwi_pick_config.txt'):
                (root / name).touch()
            module = root / 'linux-module'
            if linux:
                module.mkdir()
            source = (ROOT / 'run_dual_pick.sh').read_text().replace(
                'AXIVC_DEVICE=/dev/axivc', 'AXIVC_DEVICE=/dev/null'
            ).replace('/sys/module/axvisor', str(module))
            (root / 'run_dual_pick.sh').write_text(source)
            (root / 'run_dual_pick.sh').chmod(0o755)
            (root / 'run_dual_pick_ci_once.sh').write_text(
                (ROOT / 'run_dual_pick_ci_once.sh').read_text())
            executable = root / 'bin/tennis-perception'
            threshold = reported_threshold or (args[1] if len(args) == 2 else '15')
            log = 'echo "ROBOT_CONFIG_APPLIED session=17 chunks=9 crc=0x1234"\n'
            for window in range(1, windows + 1):
                log += (f'echo "STARRY_ROBOT_CI_PERF_WINDOW index={window}/2 '
                        f'elapsed_ms=10000 inferences=300 effective_fps={fps} '
                        f'threshold={threshold}"\n')
            if complete:
                if control:
                    log += (f'echo "ROBOT_CONTROL_DONE session={control_session} '
                            f'status={control_status} cycles=1 checks=7"\n')
                log += f'echo "STARRY_ROBOT_CI_DONE perf={perf} duration_ms={duration}"\n'
            executable.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n' + log +
                                  f'exit {status}\n')
            executable.chmod(0o755)
            env = dict(os.environ)
            env.pop('_AKA_DUAL_PICK_CI_ONCE', None)
            env.pop('_AKA_DUAL_PICK_CI_MIN_FPS', None)
            script = 'run_dual_pick_ci_once.sh' if ci else 'run_dual_pick.sh'
            return subprocess.run(['sh', str(root / script), *args], env=env,
                                  capture_output=True, text=True, timeout=5)

    def test_explicit_gate_is_forwarded_independently_of_guest_detection(self):
        for linux in (True, False):
            for threshold in ('28', '19', '23.5'):
                with self.subTest(linux=linux, threshold=threshold):
                    result = self.launch(linux, args=('--min-fps', threshold))
                    self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
                    self.assertIn('DUAL_PICK_CI_CHECK_PASS', result.stdout)
                    self.assertNotIn('ZEPHYR_', result.stdout)
                    self.assertIn('--robot-ci-once\n--min-fps\n' + threshold + '\n',
                                  result.stdout)

    def test_ci_requires_a_finite_positive_explicit_gate(self):
        for args in ((), ('--min-fps',), ('--wrong', '28'), ('--min-fps', 'NaN'),
                     ('--min-fps', '0'), ('--min-fps', '-1'), ('--min-fps', '1001'),
                     ('--min-fps', '28.001'), ('--min-fps', '19', '--extra')):
            with self.subTest(args=args):
                result = self.launch(False, args=args)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('STARRY_ROBOT_CI_PERF_WINDOW', result.stdout)

    def test_success_requires_both_windows_and_full_completion_on_publisher(self):
        for case in ({'complete': False}, {'windows': 1}, {'fps': '27.99'},
                     {'reported_threshold': '15'}, {'duration': '61000'},
                     {'perf': 'fail'}, {'windows': 3}, {'fps': 'NaN'}):
            with self.subTest(case=case):
                result = self.launch(True, args=('--min-fps', '28'), **case)
                self.assertNotEqual(result.returncode, 0)

    def test_manual_run_has_no_ci_gate_and_failure_propagates(self):
        self.assertNotIn('--min-fps', self.launch(True, ci=False).stdout)
        self.assertEqual(self.launch(False, status=7, args=('--min-fps', '19')).returncode, 7)

    def test_control_confirmation_must_match_current_config_session(self):
        for case in ({'control': False}, {'control_session': '16'}, {'control_status': 'failed'}):
            with self.subTest(case=case):
                self.assertNotEqual(self.launch(False, args=('--min-fps', '19'), **case).returncode, 0)


if __name__ == '__main__':
    unittest.main()
