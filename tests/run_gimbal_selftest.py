"""主机自检：真实 SMC、云台、电机驱动和滤波，仅替换 CAN、时间和 CMSIS 边界。"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
parser.add_argument("--trace", type=Path, help="可选：保存每拍角度、积分、输出，供重构前后对比")
args = parser.parse_args()
compiler = shutil.which(args.cxx)
if compiler is None:
    parser.error("找不到本机 C++ 编译器，请用 --cxx 指定 MinGW g++.exe")

root = Path(__file__).resolve().parents[1]
output = root / "build" / "host_tests"
output.mkdir(parents=True, exist_ok=True)
executable = output / ("gimbal_selftest.exe" if os.name == "nt" else "gimbal_selftest")
includes = [
    "tests/host_include",
    "User_File/Application/Gimbal",
    "User_File/Device/Peripheral/Remote/DT7",
    "User_File/Device/Peripheral/Motor/DJImotor",
    "User_File/Middleware/BSP/CAN",
    "User_File/Middleware/Algorithm/SMC",
    "User_File/Middleware/Algorithm/PID",
    "User_File/Middleware/Algorithm/VQF",
    "User_File/Middleware/Algorithm/Matrix",
    "User_File/System/Timestamp",
]
sources = [
    "tests/test_gimbal.cpp",
    "User_File/Application/Gimbal/gimbal.cpp",
    "User_File/Application/Gimbal/gimbal_remote.cpp",
    "User_File/Middleware/Algorithm/SMC/alg_smc.cpp",
    "User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp",
    "User_File/Middleware/Algorithm/PID/alg_pid.cpp",
    "User_File/Middleware/Algorithm/PID/alg_filter_iir.cpp",
    "User_File/Middleware/Algorithm/VQF/alg_basic.cpp",
    "User_File/Middleware/Algorithm/Matrix/alg_matrix_cmsis_compat.c",
]
environment = os.environ.copy()
environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
command = [compiler, "-std=c++17", "-O0", "-g", "-Wall", "-Wextra", "-Werror"]
command += ["-I" + str(root / path) for path in includes]
command += [str(root / path) for path in sources] + ["-o", str(executable)]
subprocess.run(command, check=True, env=environment)
subprocess.run([str(executable)] + ([str(args.trace)] if args.trace else []),
               check=True, env=environment)
