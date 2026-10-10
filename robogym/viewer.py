"""Streams an environment's default view to browsers as H.264, for viewer.html.

Each frame goes out as a 4-byte little-endian length, a keyframe flag byte, and the
frame in H.264's Annex B format.
"""

import asyncio
import struct
from collections.abc import AsyncIterator, Awaitable, Callable
from fractions import Fraction

import av
import numpy as np

from robogym.env import VIEW_HEIGHT, VIEW_WIDTH

FPS = 30
KEYFRAME_INTERVAL = 2 * FPS  # a browser starts decoding at a keyframe


class Viewer:
    """Sends each new view to every browser watching, at most FPS times a second."""

    def __init__(self, view: Callable[[], Awaitable[np.ndarray | None]]):
        self._view = view
        self._encoder = _h264_encoder()
        self._watchers: set[_Watcher] = set()
        self._since_keyframe: list[bytes] = []  # what a joining browser needs first
        self._ended = False

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            start = loop.time()
            if self._watchers and (image := await self._view()) is not None:
                self._send(*await asyncio.to_thread(self._encode, image))
            await asyncio.sleep(start + 1 / FPS - loop.time())

    async def watch(self) -> AsyncIterator[bytes]:
        """One browser's frames: those since the last keyframe, then each new one."""
        watcher = _Watcher(self._since_keyframe)
        self._watchers.add(watcher)
        try:
            while not self._ended and (frame := await watcher.frames.get()):
                yield frame
        finally:
            self._watchers.discard(watcher)

    def end_streams(self) -> None:
        """End every browser's stream, and any that start later."""
        self._ended = True
        for watcher in self._watchers:
            watcher.frames.put_nowait(b"")
        self._watchers.clear()

    def _encode(self, image: np.ndarray) -> tuple[bytes, bool]:
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        (packet,) = self._encoder.encode(frame)  # zerolatency: one packet per frame
        data = bytes(packet)
        header = struct.pack("<IB", len(data), packet.is_keyframe)
        return header + data, packet.is_keyframe

    def _send(self, frame: bytes, keyframe: bool) -> None:
        if keyframe:
            self._since_keyframe.clear()
        self._since_keyframe.append(frame)
        for watcher in self._watchers:
            watcher.put(frame, keyframe)


class _Watcher:
    """The frames queued for one browser."""

    def __init__(self, frames: list[bytes]):
        self.frames: asyncio.Queue[bytes] = asyncio.Queue()
        for frame in frames:
            self.frames.put_nowait(frame)
        self._skipping = False

    def put(self, frame: bytes, keyframe: bool) -> None:
        """Queue a frame, unless this browser is skipping to the next keyframe."""
        if self.frames.qsize() > KEYFRAME_INTERVAL:  # too far behind
            while not self.frames.empty():
                self.frames.get_nowait()
            self._skipping = True
        if keyframe:
            self._skipping = False
        if not self._skipping:
            self.frames.put_nowait(frame)


def _h264_encoder() -> av.CodecContext:
    """libx264 tuned for latency, at the avc1.64001f profile viewer.html decodes."""
    encoder = av.CodecContext.create("libx264", "w")
    encoder.width, encoder.height = VIEW_WIDTH, VIEW_HEIGHT
    encoder.pix_fmt = "yuv420p"
    encoder.time_base = Fraction(1, FPS)
    encoder.gop_size = KEYFRAME_INTERVAL
    encoder.options = {
        "preset": "veryfast",
        "tune": "zerolatency",
        "crf": "23",
        "profile": "high",
        "level": "3.1",
    }
    return encoder
