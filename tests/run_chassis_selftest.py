"""编译真实底盘和 DT7 代码，只替换 UART 接收、时间与 CMSIS 边界。"""
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
executable = output / ("chassis_selftest.exe" if os.name == "nt" else "chassis_selftest")
includes = [
    "tests/host_include",
    "User_File/Application/Chassis",
    "User_File/Device/Peripheral/Remote/DT7",
    "User_File/Middleware/BSP/UART",
    "User_File/System/Timestamp",
]
sources = [
    "tests/test_chassis.cpp",
    "User_File/Application/Chassis/chassis.cpp",
    "User_File/Device/Peripheral/Remote/DT7/dt7.cpp",
]
environment = os.environ.copy()
environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
command = [compiler, "-std=c++17", "-O0", "-g", "-Wall", "-Wextra", "-Werror"]
command += ["-I" + str(root / path) for path in includes]
command += [str(root / path) for path in sources] + ["-o", str(executable)]
subprocess.run(command, check=True, env=environment)
subprocess.run([str(executable)], check=True, env=environment)
