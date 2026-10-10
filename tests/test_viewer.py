"""Tests for the viewer, streaming views made up for the purpose."""

import asyncio
import struct

import av
import numpy as np

from robogym.env import VIEW_HEIGHT, VIEW_WIDTH
from robogym.viewer import KEYFRAME_INTERVAL, Viewer, _Watcher


class Views:
    """A view of one shade of grey, new whenever the shade changes."""

    def __init__(self):
        self.shade, self._shown = 0, None

    async def __call__(self) -> np.ndarray | None:
        if self.shade == self._shown:
            return None
        self._shown = self.shade
        return np.full((VIEW_HEIGHT, VIEW_WIDTH, 3), self.shade, np.uint8)


def stream(scenario) -> None:
    """Run `await scenario(viewer, views)` while a viewer streams views."""

    async def main():
        views = Views()
        viewer = Viewer(views)
        streaming = asyncio.create_task(viewer.run())
        try:
            await scenario(viewer, views)
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


def test_a_browser_gets_the_view_at_once():
    async def scenario(viewer, views):
        frames = watch(viewer)
        await asyncio.sleep(0.2)
        assert len(frames) == 1 and is_keyframe(frames[0])
        (image,) = decode(frames)
        assert image.shape == (VIEW_HEIGHT, VIEW_WIDTH, 3)

    stream(scenario)


def test_a_frame_comes_only_when_the_view_changes():
    async def scenario(viewer, views):
        frames = watch(viewer)
        await asyncio.sleep(0.2)
        views.shade = 200
        await asyncio.sleep(0.2)
        assert len(frames) == 2
        dark, light = decode(frames)
        assert dark.mean() < 10 and abs(light.mean() - 200) < 10

    stream(scenario)


def test_a_browser_joining_later_starts_at_the_last_keyframe():
    async def scenario(viewer, views):
        first = watch(viewer)
        for shade in range(5):
            views.shade = 40 * shade
            await asyncio.sleep(0.1)
        later = watch(viewer)
        await asyncio.sleep(0.2)
        assert is_keyframe(later[0]) and later == first[-len(later) :]
        assert len(decode(later)) == len(later)

    stream(scenario)


def test_ending_the_streams_ends_those_watching_and_those_to_come():
    async def scenario(viewer, views):
        watching = viewer.watch()
        assert is_keyframe(await anext(watching))
        viewer.end_streams()
        assert [frame async for frame in watching] == []
        assert [frame async for frame in viewer.watch()] == []

    stream(scenario)


def test_a_browser_too_far_behind_skips_to_the_next_keyframe():
    watcher = _Watcher([])
    for _ in range(KEYFRAME_INTERVAL + 2):
        watcher.put(b"delta", keyframe=False)
    watcher.put(b"key", keyframe=True)
    assert watcher.frames.qsize() == 1 and watcher.frames.get_nowait() == b"key"
