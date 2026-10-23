"""Exercise CMake packaging, incremental Python sync and exported C++ requirements."""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import zipfile


ROOT = Path(__file__).resolve().parents[2]
PACKAGES = ("csim_control", "csim_experiments", "csim_bridge", "csim_viewer")


def run(command, *, cwd, env=None):
    subprocess.run([str(argument) for argument in command], cwd=cwd, env=env, check=True)


def python_check(code, directory, *, cwd):
    env = dict(os.environ, PYTHONPATH=str(directory))
    run([sys.executable, "-c", code], cwd=cwd, env=env)


def check(generator, parallel):
    with tempfile.TemporaryDirectory(prefix="csim-cmake-regression-") as temporary:
        workspace = Path(temporary)
        distributions = workspace / "dist"
        run([sys.executable, ROOT / "setup.py", "egg_info", "--egg-base", workspace,
             "sdist", "--dist-dir", distributions], cwd=ROOT)
        archive_path, = distributions.glob("*.tar.gz")
        with tarfile.open(archive_path) as archive:
            names = archive.getnames()
            prefix = names[0].split("/")[0]
            for required in ("CMakeLists.txt", "cmake/CSimConfig.cmake.in", "cmake/SyncDirectory.cmake"):
                if f"{prefix}/{required}" not in names:
                    raise AssertionError(f"Source distribution is missing {required}")
            if any(name.startswith(f"{prefix}/docs/") for name in names):
                raise AssertionError("Local docs must not be packaged")
            extraction_options = {"filter": "data"} if hasattr(tarfile, "data_filter") else {}
            archive.extractall(workspace, **extraction_options)
        source = workspace / prefix

        env = dict(os.environ, CSIM_BUILD_SYSTEM="cmake", CSIM_BUILD_VIEWER="1",
                   CMAKE_GENERATOR=generator, CMAKE_BUILD_PARALLEL_LEVEL=str(parallel))
        wheels = workspace / "wheels"
        run([sys.executable, "setup.py", "bdist_wheel", "--dist-dir", wheels], cwd=source, env=env)
        wheel_path, = wheels.glob("*.whl")
        installed = workspace / "wheel"
        with zipfile.ZipFile(wheel_path) as wheel:
            wheel.extractall(installed)
        python_check(
            "import csim, csim_control, csim_experiments, csim_bridge, csim_viewer, _csim_viewer; "
            "model = csim.DroneModel(); data = csim.make_data(model); "
            "csim.step(model, data); assert csim.get_state(model, data)['time'] > 0",
            installed, cwd=workspace)

        build = workspace / "build"
        pybind_directory = subprocess.check_output(
            [sys.executable, "-m", "pybind11", "--cmakedir"], text=True).strip()
        run(["cmake", "-S", source, "-B", build, "-G", generator,
             "-DCMAKE_BUILD_TYPE=Release", "-DCSIM_BUILD_TESTS=OFF",
             "-DCSIM_BUILD_PYTHON=ON", "-DCSIM_BUILD_VIEWER=ON",
             "-DCSIM_BUILD_PYTHON_TESTS=OFF", "-DCSIM_BUILD_BENCHMARKS=OFF",
             f"-DPython3_EXECUTABLE={sys.executable}", f"-Dpybind11_DIR={pybind_directory}"], cwd=workspace)
        build_command = ["cmake", "--build", build, "--config", "Release", "--parallel", parallel]
        run(build_command, cwd=workspace)
        output = build / "Release" if generator == "Ninja Multi-Config" else build
        extensions = [path for path in output.iterdir() if path.suffix in (".so", ".pyd")]
        if len(extensions) != 2:
            raise AssertionError(f"Expected two Python extensions in {output}")
        original_times = {path: path.stat().st_mtime_ns for path in extensions}
        probes = [source / "bindings/python" / package / "_cmake_sync_probe.py" for package in PACKAGES]
        incremental_build = build_command + ["--target", "csim", "csim_python_viewer", "viewer"]
        shader = source / "bindings/python/csim_viewer/shaders/_cmake_sync_probe.vert"
        for value in (1, 2):
            for probe in probes:
                probe.write_text(f"VALUE = {value}\n")
            shader.write_text(f"probe {value}\n")
            run(incremental_build, cwd=workspace)
            for package in PACKAGES:
                python_check(f"from {package} import _cmake_sync_probe; "
                             f"assert _cmake_sync_probe.VALUE == {value}", output, cwd=workspace)
            for directory in (output / "shaders", output / "csim_viewer/shaders"):
                if (directory / shader.name).read_text() != shader.read_text():
                    raise AssertionError("Viewer shaders were not refreshed")
        for probe in probes:
            probe.unlink()
        shader.unlink()
        run(incremental_build, cwd=workspace)
        for package in PACKAGES:
            python_check(f"import importlib.util; "
                         f"assert importlib.util.find_spec('{package}._cmake_sync_probe') is None",
                         output, cwd=workspace)
        for directory in (output / "shaders", output / "csim_viewer/shaders"):
            if (directory / shader.name).exists():
                raise AssertionError("Deleted viewer shader remains in the build tree")
        if any(path.stat().st_mtime_ns != stamp for path, stamp in original_times.items()):
            raise AssertionError("Python-only edits must not relink the extensions")

        prefix = workspace / "install"
        run(["cmake", "--install", build, "--config", "Release", "--prefix", prefix], cwd=workspace)
        consumer = workspace / "consumer"
        consumer.mkdir()
        (consumer / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.20)\n"
            "project(CSimConsumer LANGUAGES CXX)\n"
            "set(CMAKE_CXX_STANDARD 14)\n"
            "set(CMAKE_CXX_STANDARD_REQUIRED ON)\n"
            "find_package(CSim CONFIG REQUIRED)\n"
            "add_executable(consumer main.cpp)\n"
            "target_link_libraries(consumer PRIVATE CSim::csim_simulation CSim::csim_io)\n")
        (consumer / "main.cpp").write_text(
            "#include <csim/simulation/drone.hpp>\n"
            "static_assert(__cplusplus >= 201703L);\n"
            "int main() {\n"
            "    auto model = std::make_shared<csim::simulation::DroneModel>();\n"
            "    auto data = csim::simulation::makeData(model, {});\n"
            "    csim::simulation::step(*model, data);\n"
            "}\n")
        consumer_build = consumer / "build"
        run(["cmake", "-S", consumer, "-B", consumer_build, "-G", "Ninja",
             f"-DCMAKE_PREFIX_PATH={prefix}"], cwd=workspace)
        run(["cmake", "--build", consumer_build], cwd=workspace)
        run([consumer_build / "consumer"], cwd=workspace)
    print(f"PASS: CMake packaging, Python sync and C++17 consumer ({generator})", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generator", choices=("Ninja", "Ninja Multi-Config"), default="Ninja")
    parser.add_argument("--parallel", type=int, default=2)
    arguments = parser.parse_args()
    check(arguments.generator, arguments.parallel)
