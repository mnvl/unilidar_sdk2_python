import platform
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

ROOT = Path(__file__).resolve().parent


def _normalize_arch(machine: str) -> str:
    m = machine.lower()
    if m in {"x86_64", "amd64"}:
        return "x86_64"
    if m in {"aarch64", "arm64"}:
        return "aarch64"
    raise RuntimeError(
        f"Unsupported architecture '{machine}'. Expected one of: x86_64, aarch64"
    )


arch = _normalize_arch(platform.machine())
static_lib = ROOT / "lib" / arch / "libunilidar_sdk2.a"
if not static_lib.exists():
    raise RuntimeError(f"Missing SDK static library: {static_lib}")

ext_modules = [
    Pybind11Extension(
        "unitree_lidar_sdk._native",
        ["src/bindings.cpp"],
        include_dirs=[str(ROOT / "include")],
        extra_objects=[str(static_lib)],
        cxx_std=17,
        extra_compile_args=["-O3", "-fvisibility=hidden", "-pthread"],
        extra_link_args=["-pthread"],
    )
]

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
