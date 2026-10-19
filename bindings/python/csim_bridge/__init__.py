"""Optional bridge contracts; importing this package starts no transport."""

from importlib.metadata import entry_points

from .api import BridgePlugin, PhysicalWrench


def available_bridges():
    """List installed factories registered in the csim.bridges entry point group."""
    return tuple(sorted({entry.name for entry in entry_points(group="csim.bridges")}))


def load_bridge(name, **options):
    """Construct a plugin without starting its connection or simulation clock."""
    matches = tuple(entry for entry in entry_points(group="csim.bridges") if entry.name == name)
    if not matches:
        raise LookupError(f"Bridge {name!r} is not installed; available: {available_bridges()}")
    if len(matches) != 1:
        raise ValueError(f"Multiple installed bridges use the name {name!r}")
    plugin = matches[0].load()(**options)
    if not isinstance(plugin, BridgePlugin) or not all(
        callable(getattr(plugin, method, None))
        for method in ("start", "publish_state", "receive_wrench", "reset", "close")
    ):
        raise TypeError("Bridge must implement start, publish_state, receive_wrench, reset and close")
    return plugin


__all__ = ["BridgePlugin", "PhysicalWrench", "available_bridges", "load_bridge"]
