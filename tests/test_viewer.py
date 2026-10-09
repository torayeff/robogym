"""Tests for the viewer, streaming the SO-101 dual-arm environment's default view."""

import asyncio
import struct

import av
import numpy as np

from robogym.envs.so101_dual_arm.env import So101DualArm
from robogym.viewer import HEIGHT, KEYFRAME_INTERVAL, WIDTH, Viewer, _Watcher


def stream(env: So101DualArm, scenario) -> None:
    """Run `await scenario(viewer)` while a viewer streams env's view."""

    async def state():
        return env.state()

    async def main():
        with Viewer(env, state) as viewer:
            streaming = asyncio.create_task(viewer.run())
            try:
                await scenario(viewer)
            finally:
                streaming.cancel()

    asyncio.run(main())


def watch(viewer: Viewer) -> list[bytes]:
    """Start watching as a browser would; returns the list the frames arrive in."""
    frames = []

    async def receive():
        async for frame in viewer.watch():
            frames.append(frame)

    asyncio.create_task(receive())
    return frames


def is_keyframe(frame: bytes) -> bool:
    return frame[4] == 1


def decode(frames: list[bytes]) -> list[np.ndarray]:
    """Decode frames, the first of which must be a keyframe."""
    decoder = av.CodecContext.create("h264", "r")
    images = []
    for frame in frames:
        (size,) = struct.unpack_from("<I", frame)
        assert len(frame) == 5 + size
        for image in decoder.decode(av.Packet(frame[5:])):
            images.append(image.to_ndarray(format="rgb24"))
    return images


def test_a_browser_gets_the_view_at_once(env):
    async def scenario(viewer):
        frames = watch(viewer)
        await asyncio.sleep(0.3)
        assert len(frames) == 1 and is_keyframe(frames[0])
        (image,) = decode(frames)
        assert image.shape == (HEIGHT, WIDTH, 3)

    stream(env, scenario)


def test_a_frame_comes_only_when_something_moves(env):
    async def scenario(viewer):
        frames = watch(viewer)
        await asyncio.sleep(0.3)
        env.act({"joints": {"left_shoulder_lift": 0.0}})
        env.step(1.0)
        await asyncio.sleep(0.3)
        assert len(frames) == 2  # one before the arm moved, one after
        before, after = decode(frames)
        changed = np.abs(after.astype(int) - before).max(axis=-1) > 50
        assert changed.mean() > 0.003  # the arm is somewhere new in the picture

    stream(env, scenario)


def test_a_browser_joining_later_starts_at_the_last_keyframe(env):
    async def scenario(viewer):
        first = watch(viewer)
        env.act({"joints": {"left_shoulder_lift": 0.0}})
        for _ in range(5):
            env.step(0.1)
            await asyncio.sleep(0.1)
        later = watch(viewer)
        await asyncio.sleep(0.3)
        assert is_keyframe(later[0]) and later == first[-len(later) :]
        assert len(decode(later)) == len(later)

    stream(env, scenario)


def test_a_browser_too_far_behind_skips_to_the_next_keyframe():
    watcher = _Watcher([])
    for _ in range(KEYFRAME_INTERVAL + 2):
        watcher.put(b"delta", keyframe=False)
    watcher.put(b"key", keyframe=True)
    assert watcher.frames.qsize() == 1 and watcher.frames.get_nowait() == b"key"
