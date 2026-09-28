"""Static + guard regression for the AArch64 cross link contract.

The cross build must link the real AArch64 runtime libraries. The former
"empty stub .so" scheme and the unresolved-symbol linker flags are forbidden:
they produced a tennis binary whose uvc_* calls had no dynamic relocation and
jumped through an unresolved PLT entry (SIGILL) in UvcCapture::open.

This test only reads the build files and runs the lightweight CMake guard script
(existence check). It never configures, builds, or links anything.
"""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CMAKE_LISTS = ROOT / "CMakeLists.txt"
BUILD_SCRIPT = ROOT / "build_rk3588.sh"
GUARD_SCRIPT = ROOT / "cmake" / "require_target_runtime_libs.cmake"

# Linked explicitly by the tennis target and recorded as DT_NEEDED.
LINKED_LIBS = ("libuvc.so", "libusb-1.0.so", "libturbojpeg.so")
# Also required in the cross library directory. libudev.so.1 is a transitive
# dependency of libusb-1.0.so resolved through the -rpath-link directory, so it
# must exist there even though tennis does not link it by name.
TRANSITIVE_LIBS = ("libudev.so.1",)
REQUIRED_LIBS = LINKED_LIBS + TRANSITIVE_LIBS


def strip_comments(text, marker="#"):
    """Drop line/trailing comments so prose about forbidden flags is not matched."""
    lines = []
    for line in text.splitlines():
        index = line.find(marker)
        if index != -1:
            line = line[:index]
        lines.append(line)
    return "\n".join(lines)


class CrossLinkPolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cmake_code = strip_comments(CMAKE_LISTS.read_text(encoding="utf-8"))
        cls.script_code = strip_comments(BUILD_SCRIPT.read_text(encoding="utf-8"))
        cls.guard_code = strip_comments(GUARD_SCRIPT.read_text(encoding="utf-8"))

    def test_cmake_has_no_stub_or_unresolved_link_options(self):
        for forbidden in (
            "--unresolved-symbols",
            "--allow-shlib-undefined",
            "stub_libs",
            "stub_${STUBLIB}",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.cmake_code)

    def test_cmake_requires_real_target_library_directory(self):
        self.assertIn("TARGET_RUNTIME_LIB_DIR", self.cmake_code)
        self.assertIn("tennis_requires_target_runtime_libs", self.cmake_code)
        self.assertIn(
            "add_dependencies(tennis tennis_requires_target_runtime_libs)",
            self.cmake_code,
        )
        for lib in LINKED_LIBS:
            with self.subTest(lib=lib):
                self.assertIn(lib, self.cmake_code)

    def test_guard_requires_transitive_libraries(self):
        for lib in REQUIRED_LIBS:
            with self.subTest(lib=lib):
                self.assertIn(lib, self.guard_code)

    def test_build_script_gates_cross_build_before_configure(self):
        self.assertIn("AKA_RK3588_CROSS_LIB_DIR", self.script_code)
        self.assertIn("-DTARGET_RUNTIME_LIB_DIR=", self.script_code)
        for lib in REQUIRED_LIBS:
            with self.subTest(lib=lib):
                self.assertIn(lib, self.script_code)
        # The gate must run before the cmake configure call.
        self.assertLess(
            self.script_code.index("CROSS_RUNTIME_LIBS"),
            self.script_code.index("cmake -S"),
        )

    def run_guard(self, lib_dir=None):
        cmake = shutil.which("cmake")
        if not cmake:
            self.skipTest("cmake not available to run the guard script")
        args = [cmake]
        if lib_dir is not None:
            args.append("-DTARGET_RUNTIME_LIB_DIR=" + str(lib_dir))
        args += ["-P", str(GUARD_SCRIPT)]
        return subprocess.run(args, capture_output=True, text=True, timeout=60)

    def test_guard_fails_without_a_directory(self):
        result = self.run_guard(None)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("TARGET_RUNTIME_LIB_DIR", result.stderr + result.stdout)

    def test_guard_fails_when_a_library_is_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for lib in REQUIRED_LIBS[:-1]:
                (root / lib).touch()
            result = self.run_guard(root)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(REQUIRED_LIBS[-1], result.stderr + result.stdout)

    def test_guard_accepts_a_complete_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for lib in REQUIRED_LIBS:
                (root / lib).touch()
            result = self.run_guard(root)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)


if __name__ == "__main__":
    unittest.main()
