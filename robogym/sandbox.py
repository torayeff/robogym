"""Serves environments from Modal sandboxes, each at a public URL of Modal's."""

import tomllib
from pathlib import Path

import modal

PORT = 8000
VCPUS_PER_CORE = 2
IDLE_TIMEOUT = 30 * 60
LIFETIME = 2 * 60 * 60
REPO = Path(__file__).parents[1]


def create(env_name: str) -> tuple[str, str]:
    """Serve env_name from a new sandbox, and return its ID and URL once it serves."""
    package = Path(__file__).with_name("envs") / env_name
    requirements = tomllib.loads((package / "sandbox.toml").read_text())
    command = ["robogym", "serve", env_name, "--host", "0.0.0.0", "--port", str(PORT)]
    with modal.enable_output():
        sandbox = modal.Sandbox.create(
            *command,
            app=_app(),
            tags={"env": env_name},
            image=modal.Image.from_dockerfile(package / "Dockerfile", context_dir=REPO),
            cpu=requirements["vcpus"] / VCPUS_PER_CORE,
            memory=requirements["memory_gib"] * 1024,
            gpu=requirements.get("gpu"),
            runtime="gvisor" if "gpu" in requirements else "vm",
            encrypted_ports=[PORT],
            readiness_probe=modal.Probe.with_tcp(PORT),
            timeout=LIFETIME,
            idle_timeout=IDLE_TIMEOUT,
        )
    sandbox.wait_until_ready()
    return sandbox.object_id, sandbox.tunnels()[PORT].url


def running() -> list[dict[str, str]]:
    """The id, env and url of each sandbox that runs."""
    return [
        {
            "id": sandbox.object_id,
            **sandbox.get_tags(),
            "url": sandbox.tunnels()[PORT].url,
        }
        for sandbox in modal.Sandbox.list(app_id=_app().app_id)
    ]


def delete(sandbox_id: str) -> bool:
    """Stop the sandbox with this ID; False if no such sandbox runs."""
    for sandbox in modal.Sandbox.list(app_id=_app().app_id):
        if sandbox.object_id == sandbox_id:
            sandbox.terminate()
            return True
    return False


def _app() -> modal.App:
    return modal.App.lookup("robogym", create_if_missing=True)
