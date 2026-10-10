"""The robogym command, which serves an environment over HTTP."""

import argparse

from robogym.envs import ENVS


def main() -> None:
    parser = argparse.ArgumentParser(prog="robogym", description=__doc__)
    parser.add_argument("env", choices=ENVS, help="the environment to serve")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    # Imported here: the physics process re-imports this module and needs no server.
    from robogym.server import serve

    serve(args.env, args.host, args.port)


if __name__ == "__main__":
    main()
