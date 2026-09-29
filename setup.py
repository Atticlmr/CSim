"""PEP 517/660 packaging bridge; xmake remains the C++ build system."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import sysconfig
import tempfile

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


ROOT = Path(__file__).resolve().parent
BUILD_VIEWER = os.environ.get("CSIM_BUILD_VIEWER", "1") != "0"
# Keep release metadata aligned with the existing xmake project version.
VERSION = re.search(r'^set_version\("([^"]+)"\)',
                    (ROOT / "xmake.lua").read_text(), re.MULTILINE).group(1)


class XmakeBuildExt(build_ext):
    def run(self):
        # A prior full build must not leak a viewer into a headless wheel.
        if not BUILD_VIEWER:
            for path in Path(self.build_lib).glob("_csim_viewer*.so"):
                path.unlink()
            for path in (Path(self.build_lib) / "csim_viewer" / ".libs").glob("*glfw*"):
                path.unlink()
        super().run()

    def build_extension(self, ext):
        if sys.platform not in ("linux", "darwin"):
            raise RuntimeError("CSim source builds currently support native Linux/macOS only.")
        if os.environ.get("ARCHFLAGS") or os.environ.get("_PYTHON_HOST_PLATFORM"):
            raise RuntimeError("CSim wheels currently require a native, single-architecture build.")
        xmake = shutil.which(os.environ.get("CSIM_XMAKE", "xmake"))
        if not xmake:
            raise RuntimeError(
                "xmake >= 3.1.1 is required to build CSim from source. "
                "Install xmake from https://xmake.io/guide/quick-start.html, "
                "set CSIM_XMAKE to its executable, or install a prebuilt CSim wheel."
            )

        destination = Path(self.get_ext_fullpath(ext.name)).resolve()
        # Build a fresh staged project: pip must not reconfigure the developer's
        # .xmake cache, pick up an old .so, or mix Python ABIs in build/.
        with tempfile.TemporaryDirectory(prefix="csim-wheel-") as directory:
            staging = Path(directory)
            project = staging / "project"
            project.mkdir()
            shutil.copy2(ROOT / "xmake.lua", project / "xmake.lua")
            for name in ("include", "src", "bindings", "apps", "tests", "examples"):
                shutil.copytree(ROOT / name, project / name,
                                ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.so", "*.pyd"))
            env = dict(os.environ,
                       XMAKE_GLOBALDIR=str(staging / "global"),
                       XMAKE_CONFIGDIR=str(staging / "config"),
                       XMAKE_COLORTERM="nocolor")
            # Preserve pip's build-isolation environment for build_info.py so
            # it sees the build-required pybind11, using the installing Python.
            subprocess.run([xmake, "f", "-y", "-m", "release",
                            "--viewer=" + ("y" if ext.name == "_csim_viewer" else "n"),
                            "--python=y", f"--python_executable={sys.executable}",
                            "--ccache=n"], cwd=project, env=env, check=True)
            subprocess.run([xmake, "build", "-y", "-j", "2",
                            "csim_python_viewer" if ext.name == "_csim_viewer" else "csim_python"],
                           cwd=project, env=env, check=True)
            suffix = sysconfig.get_config_var("EXT_SUFFIX")
            candidates = list((project / "build").rglob(ext.name + suffix))
            if len(candidates) != 1:
                raise RuntimeError(f"Expected one CSim extension for {suffix}, found {candidates}")
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(candidates[0], destination)
            if sys.platform == "linux":
                dependencies = subprocess.check_output(["ldd", str(candidates[0])], text=True)
                for line in dependencies.splitlines():
                    match = re.match(r"\s*(lib(?:glfw|tinyxml2)\.so[^ ]*) => (/[^ ]+)", line)
                    if match:
                        libraries = destination.parent / "csim_viewer" / ".libs"
                        libraries.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(match.group(2), libraries / match.group(1))
                        package = "libglfw3" if "glfw" in match.group(1) else "libtinyxml2-" + match.group(1).split(".so.")[-1]
                        license_file = Path("/usr/share/doc") / package / "copyright"
                        if not license_file.exists():
                            raise RuntimeError(f"Bundled library license missing: {license_file}")
                        shutil.copy2(license_file, libraries / (package + "-LICENSE"))


setup(version=VERSION,
      ext_modules=[Extension("csim", sources=["bindings/python/module.cpp"])] +
                  ([Extension("_csim_viewer", sources=["bindings/python/viewer_module.cpp"])] if BUILD_VIEWER else []),
      cmdclass={"build_ext": XmakeBuildExt})
