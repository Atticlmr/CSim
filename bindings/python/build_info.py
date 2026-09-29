"""Describe the selected interpreter and its installed pybind11 headers."""

import json
from pathlib import Path
import sysconfig

try:
    import pybind11
except ImportError as error:
    raise SystemExit(
        "pybind11 is missing from the selected Python interpreter. "
        "Install python3-pybind11 (Ubuntu system Python), or install pybind11 "
        "in your virtual environment. See docs/python.md."
    ) from error

include_dirs = list(dict.fromkeys([
    sysconfig.get_path("include"),
    sysconfig.get_path("platinclude"),
    pybind11.get_include(),
]))
if not (Path(include_dirs[0]) / "Python.h").is_file():
    raise SystemExit("Python.h is missing; install development headers for the selected Python.")
if not (Path(pybind11.get_include()) / "pybind11" / "pybind11.h").is_file():
    raise SystemExit("The selected Python's pybind11 headers were not found.")
extension = sysconfig.get_config_var("EXT_SUFFIX")
if not extension:
    raise SystemExit("The selected Python does not report an extension module suffix.")

print(json.dumps({"include_dirs": include_dirs, "extension": extension}))
