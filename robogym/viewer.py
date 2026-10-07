"""Streams an environment's default view to browsers as H.264, for viewer.html to show.

A render thread renders and encodes each frame once for all browsers, so the event loop
only copies positions and fans out bytes. A frame is made at most FPS times a second,
and only when something has moved.

Each frame goes out as a 4-byte little-endian length, a keyframe flag byte, and the
frame in H.264's Annex B format.
"""

import asyncio
import concurrent.futures
import struct
from collections.abc import AsyncIterator
from fractions import Fraction

import av
import numpy as np

from robogym.env import Env

WIDTH, HEIGHT = 1280, 720
FPS = 30
# A browser starts from a keyframe, so there is one every two seconds' worth of frames.
KEYFRAME_INTERVAL = 2 * FPS
# A frame is made when a position has moved more than this since the last one.
MIN_MOVE = 1e-6


class Viewer:
    """Sends env's default view to every browser watching, whenever it changes.

    Make and close it on the environment's thread, and run() it on that thread's event
    loop.
    """

    def __init__(self, env: Env):
        self._env = env
        self._renderer = env.view_renderer(WIDTH, HEIGHT)
        self._encoder = _h264_encoder()
        # Renders and encodes every frame, so it owns the renderer's OpenGL context.
        self._render_thread = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self._watchers: set[_Watcher] = set()
        # The frames since the last keyframe, which a joining browser needs first.
        self._since_keyframe: list[bytes] = []

    async def run(self) -> None:
        """Send a frame whenever the view changes, at most FPS times a second."""
        loop = asyncio.get_running_loop()
        sent = None  # the positions of the last frame sent
        while True:
            start = loop.time()
            if self._watchers:
                positions = self._env.positions()
                if sent is None or np.any(np.abs(positions - sent) > MIN_MOVE):
                    frame, keyframe = await loop.run_in_executor(
                        self._render_thread, self._encode, positions
                    )
                    self._send(frame, keyframe)
                    sent = positions
            await asyncio.sleep(start + 1 / FPS - loop.time())

    async def watch(self) -> AsyncIterator[bytes]:
        """One browser's frames: those since the last keyframe, then each new one."""
        watcher = _Watcher(self._since_keyframe)
        self._watchers.add(watcher)
        try:
            while True:
                yield await watcher.frames.get()
        finally:
            self._watchers.discard(watcher)

    def close(self) -> None:
        """Close the renderer on the render thread, then stop the thread."""
        self._render_thread.submit(self._renderer.close)
        self._render_thread.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _encode(self, positions: np.ndarray) -> tuple[bytes, bool]:
        """Render and encode one frame, on the render thread."""
        image = self._renderer.render(positions)
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        (packet,) = self._encoder.encode(frame)  # zerolatency: one packet per frame
        data = bytes(packet)
        header = struct.pack("<IB", len(data), packet.is_keyframe)
        return header + data, packet.is_keyframe

    def _send(self, frame: bytes, keyframe: bool) -> None:
        """Queue a frame for every browser watching, and keep it for those to come."""
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
        if self.frames.qsize() > KEYFRAME_INTERVAL:
            # Too far behind: drop the backlog and resume at the next keyframe.
            while not self.frames.empty():
                self.frames.get_nowait()
            self._skipping = True
        if keyframe:
            self._skipping = False
        if not self._skipping:
            self.frames.put_nowait(frame)


def _h264_encoder() -> av.CodecContext:
    """A libx264 encoder tuned for latency: no B-frames or lookahead.

    Its profile and level match the avc1.64001f codec string viewer.html decodes.
    """
    encoder = av.CodecContext.create("libx264", "w")
    encoder.width, encoder.height = WIDTH, HEIGHT
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
