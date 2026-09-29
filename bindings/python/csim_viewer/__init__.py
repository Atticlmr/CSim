"""Passive main-thread viewer for CSim's physical models."""
import atexit as _atexit
from pathlib import Path as _Path
import threading as _threading


def _main_thread():
    if _threading.current_thread() is not _threading.main_thread():
        raise RuntimeError("CSim viewer must be created, synced and closed on the main thread")


class Viewer:
    """Display Model/Data without advancing physics. Use as a context manager.

    sync handles events and draws synchronously. Python owns stepping and pacing.
    One window at a time. Forgotten windows are closed at interpreter exit.
    """
    def __init__(self, model, *, width=1280, height=720, hidden=False):
        _main_thread()
        try:
            from _csim_viewer import Window
        except ModuleNotFoundError as error:
            raise RuntimeError("Viewer extension is unavailable; reinstall csim-drone with CSIM_BUILD_VIEWER=1") from error
        self._window = Window(model, str(_Path(__file__).parent / "shaders"), width, height, hidden)
        # Retain until close/exit so GC on another thread cannot destroy GLFW.
        _atexit.register(self.close)

    def sync(self, data):
        _main_thread()
        self._window.sync(data)

    def is_running(self):
        _main_thread()
        return self._window.is_running()

    def close(self):
        _main_thread()
        self._window.close()
        _atexit.unregister(self.close)

    def screenshot(self, path):
        """Write the most recently synced framebuffer to a P6 PPM file."""
        _main_thread()
        self._window.screenshot(str(path))

    def __enter__(self):
        _main_thread()
        if not self.is_running():
            raise RuntimeError("Viewer is closed")
        return self

    def __exit__(self, *_):
        self.close()


__all__ = ["Viewer"]
