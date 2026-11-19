"""Host check using the production BMI088 mounting transform and VQF."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
args = parser.parse_args()
compiler = shutil.which(args.cxx)
if compiler is None:
    parser.error("C++ compiler not found; specify --cxx with the MinGW g++.exe path")

root = Path(__file__).resolve().parents[1]
output = root / "build" / "host_tests"
output.mkdir(parents=True, exist_ok=True)
executable = output / ("imu_mounting_selftest.exe" if os.name == "nt" else "imu_mounting_selftest")
includes = [
    "User_File/Device/Onboard/BMI088",
    "User_File/Middleware/Algorithm/VQF",
    "User_File/Middleware/Algorithm/Matrix",
]
sources = [
    "tests/test_imu_mounting.cpp",
    "User_File/Middleware/Algorithm/VQF/alg_filter_vqf.cpp",
    "User_File/Middleware/Algorithm/VQF/alg_quaternion.cpp",
    "User_File/Middleware/Algorithm/VQF/alg_matrix.cpp",
    "User_File/Middleware/Algorithm/VQF/alg_basic.cpp",
    "User_File/Middleware/Algorithm/Matrix/alg_matrix_cmsis_compat.c",
]
environment = os.environ.copy()
environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
command = [compiler, "-std=c++17", "-O0", "-g", "-Wall", "-Wextra", "-Werror"]
command += ["-I" + str(root / path) for path in includes]
command += [str(root / path) for path in sources] + ["-o", str(executable)]
subprocess.run(command, check=True, env=environment)
subprocess.run([str(executable)], check=True, env=environment)
