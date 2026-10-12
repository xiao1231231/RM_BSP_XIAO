"""用本机 C++ 编译器运行真实 DJI 驱动/PID，自检不接入固件构建。"""
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
    parser.error("找不到本机 C++ 编译器，请用 --cxx 指定 MinGW g++.exe")

root = Path(__file__).resolve().parents[1]
output = root / "build" / "host_tests"
output.mkdir(parents=True, exist_ok=True)
executable = output / ("dji_motor_selftest.exe" if os.name == "nt" else "dji_motor_selftest")
includes = [
    "tests/host_include",
    "User_File/Device/Peripheral/Motor/DJImotor",
    "User_File/Middleware/BSP/CAN",
    "User_File/Middleware/Algorithm/PID",
    "User_File/Middleware/Algorithm/VQF",
    "User_File/Middleware/Algorithm/Matrix",
    "User_File/System/Timestamp",
]
sources = [
    "tests/test_dji_motor.cpp",
    "User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp",
    "User_File/Middleware/Algorithm/PID/alg_pid.cpp",
    "User_File/Middleware/Algorithm/PID/alg_filter_iir.cpp",
    "User_File/Middleware/Algorithm/VQF/alg_basic.cpp",
    "User_File/Middleware/Algorithm/Matrix/alg_matrix_cmsis_compat.c",
]
environment = os.environ.copy()
environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
command = [compiler, "-std=c++17", "-O0", "-g", "-Wall", "-Wextra"]
command += ["-I" + str(root / path) for path in includes]
command += [str(root / path) for path in sources] + ["-o", str(executable)]
subprocess.run(command, check=True, env=environment)
subprocess.run([str(executable)], check=True, env=environment)
