FROM ghcr.io/astral-sh/uv:0.12.3-python3.12-trixie-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libegl1 libegl-mesa0 libgl1 libgl1-mesa-dri libopengl0 \
    && rm -rf /var/lib/apt/lists/*

ENV MUJOCO_GL=egl UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy PATH=/robogym/.venv/bin:$PATH
WORKDIR /robogym
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --no-dev --no-install-project
COPY robogym robogym
RUN uv sync --frozen --no-dev
