"""Environment-indexed random streams and constraint-preserving batch resets."""

from contextlib import contextmanager
from copy import deepcopy
import math
import operator
from secrets import randbits
from threading import RLock

import numpy as np

import csim


def _integer(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be an integer, not a boolean")
    try:
        return operator.index(value)
    except TypeError as error:
        raise ValueError(f"{name} must be an integer") from error


def _seed(value):
    if value is None:
        return randbits(128)
    value = _integer(value, "seed")
    if not 0 <= value < 2**128:
        raise ValueError("seed must be in [0, 2**128)")
    return value


def _shape(size):
    if isinstance(size, (int, np.integer, bool, np.bool_)):
        size = (size,)
    try:
        shape = tuple(_integer(value, "size") for value in size)
    except TypeError as error:
        raise ValueError("size must be an integer or a tuple of integers") from error
    if any(value < 0 for value in shape):
        raise ValueError("size must be nonnegative")
    return shape


def _bounds(low, high, name):
    lower, upper = np.asarray(low, dtype=float), np.asarray(high, dtype=float)
    if lower.shape != (3,) or upper.shape != (3,):
        raise ValueError(f"{name} bounds must contain three values")
    with np.errstate(over="ignore", invalid="ignore"):
        span = upper - lower
    if not np.isfinite(lower).all() or not np.isfinite(upper).all() or not np.isfinite(span).all():
        raise ValueError(f"{name} bounds and their span must be finite")
    if np.any(lower > upper):
        raise ValueError(f"{name} lower bounds must not exceed upper bounds")
    return lower, upper


class RandomStreams:
    """PCG64 streams keyed by environment id, independent of batch size and threads."""

    def __init__(self, num_envs, seed=None):
        self._count = _integer(num_envs, "num_envs")
        if self._count <= 0:
            raise ValueError("num_envs must be positive")
        self._lock = RLock()
        self.seed(seed)

    @property
    def num_envs(self):
        return self._count

    @property
    def metadata(self):
        with self._lock:
            return {"seed": self._seed, "num_envs": self._count, "bit_generator": "PCG64",
                    "stream_key": "SeedSequence(seed, spawn_key=(environment_id,))",
                    "numpy_version": np.__version__}

    def seed(self, seed=None):
        """Restart all streams; None chooses and returns fresh 128-bit OS entropy."""
        value = _seed(seed)
        with self._lock:
            self._seed = value
            self._streams = {}
        return value

    def _selection(self, indices):
        if indices is None:
            return tuple(range(self._count))
        try:
            selected = tuple(_integer(value, "environment index") for value in indices)
        except TypeError as error:
            raise ValueError("indices must be an iterable of integers") from error
        if any(value < 0 or value >= self._count for value in selected):
            raise IndexError("Environment index out of range")
        if len(set(selected)) != len(selected):
            raise ValueError("Duplicate environment index")
        return selected

    @contextmanager
    def _transaction(self, selected, seed=None):
        with self._lock:
            root = self._seed if seed is None else _seed(seed)
            streams = dict(self._streams) if seed is None else {}
            candidates = []
            for index in selected:
                if index in streams:
                    generator = deepcopy(streams[index])
                else:
                    generator = np.random.Generator(np.random.PCG64(
                        np.random.SeedSequence(root, spawn_key=(index,))))
                streams[index] = generator
                candidates.append(generator)
            yield candidates
            self._seed = root
            self._streams = streams

    def uniform(self, low=0., high=1., *, size=(), indices=None):
        """Sample scalar bounds; the leading output axis follows selected environment ids."""
        lower, upper = float(low), float(high)
        if not math.isfinite(lower) or not math.isfinite(upper) or not math.isfinite(upper - lower):
            raise ValueError("Uniform bounds and their span must be finite")
        if lower > upper:
            raise ValueError("Uniform low must not exceed high")
        return self._sample("uniform", (lower, upper), size, indices)

    def normal(self, loc=0., scale=1., *, size=(), indices=None):
        """Sample a normal distribution without touching global Python or NumPy RNGs."""
        mean, deviation = float(loc), float(scale)
        if not math.isfinite(mean) or not math.isfinite(deviation) or deviation < 0:
            raise ValueError("Normal loc/scale must be finite and scale nonnegative")
        return self._sample("normal", (mean, deviation), size, indices)

    def _sample(self, method, parameters, size, indices):
        selected, shape = self._selection(indices), _shape(size)
        with self._transaction(selected) as generators:
            values = np.empty((len(selected), *shape))
            with np.errstate(over="ignore", invalid="ignore"):
                for row, generator in enumerate(generators):
                    values[row] = getattr(generator, method)(*parameters, size=shape)
            if not np.isfinite(values).all():
                raise OverflowError("Random samples must be finite")
        return values


class BatchRandomizer:
    """Reset DroneBatch/RigidPayloadBatch using independent streams and a fixed template."""

    def __init__(self, batch, seed=None, *, base_state=None):
        if not isinstance(batch, (csim.DroneBatch, csim.RigidPayloadBatch)):
            raise ValueError("Expected DroneBatch or RigidPayloadBatch")
        self.batch = batch
        self.random = RandomStreams(batch.num_envs, seed)
        self._rigid = isinstance(batch, csim.RigidPayloadBatch)
        states = batch.get_state()["state"] if base_state is None else base_state
        self._base = np.array(states, dtype=float, copy=True)
        columns = 27 if self._rigid else 13
        if self._base.shape != (batch.num_envs, columns) or not np.isfinite(self._base).all():
            raise ValueError(f"base_state must be finite with shape ({batch.num_envs}, {columns})")

    def seed(self, seed=None):
        """Restart the RNG only; the batch's physical state is unchanged."""
        return self.random.seed(seed)

    def reset(self, *, seed=None, indices=None, position_low=(0., 0., 0.),
              position_high=(0., 0., 0.), velocity_low=(0., 0., 0.),
              velocity_high=(0., 0., 0.)):
        """Uniformly offset template position/velocity; commit RNG only if reset succeeds."""
        if seed is not None and indices is not None:
            raise ValueError("reset(seed=...) requires a full reset; omit indices")
        selected = self.random._selection(indices)
        position = _bounds(position_low, position_high, "position")
        velocity = _bounds(velocity_low, velocity_high, "velocity")
        with self.random._transaction(selected, seed) as generators:
            states = self._base[list(selected)].copy()
            with np.errstate(over="ignore", invalid="ignore"):
                for row, generator in enumerate(generators):
                    translation = generator.uniform(*position)
                    speed = generator.uniform(*velocity)
                    states[row, :3] += translation
                    states[row, 3:6] += speed
                    if self._rigid:
                        states[row, 13:16] += translation
                        states[row, 16:19] += speed
            if not np.isfinite(states).all():
                raise OverflowError("Randomized initial state must be finite")
            self.batch.reset(states, indices=list(selected))


__all__ = ["RandomStreams", "BatchRandomizer"]
