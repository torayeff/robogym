"""The robogym command, which serves environments over HTTP, here or from sandboxes."""

import argparse

from robogym.envs import ENVS


def main() -> None:
    parser = argparse.ArgumentParser(prog="robogym", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    local = commands.add_parser("serve", help="serve an environment on this computer")
    local.add_argument("env", choices=ENVS)
    local.add_argument("--host", default="127.0.0.1")
    local.add_argument("--port", type=int, default=8000)
    remote = commands.add_parser("sandbox", help="serve from Modal sandboxes")
    actions = remote.add_subparsers(dest="action", required=True)
    create = actions.add_parser("create", help="serve an environment in a new sandbox")
    create.add_argument("env", choices=ENVS)
    actions.add_parser("list", help="list the sandboxes that run")
    delete = actions.add_parser("delete", help="stop a sandbox")
    delete.add_argument("sandbox_id")
    args = parser.parse_args()
    # Imported here: the physics process re-imports this module and needs no server.
    if args.command == "serve":
        from robogym.server import serve

        serve(args.env, args.host, args.port)
        return
    from dotenv import load_dotenv

    load_dotenv(override=True)
    from robogym import sandbox

    match args.action:
        case "create":
            sandbox_id, url = sandbox.create(args.env)
            print(f"{sandbox_id} serves {args.env} at {url}")
        case "list":
            for running in sandbox.running():
                print(f"{running['id']:<28}{running['env']:<20}{running['url']}")
        case "delete":
            if not sandbox.delete(args.sandbox_id):
                parser.exit(1, f"No sandbox {args.sandbox_id} runs.\n")


if __name__ == "__main__":
    main()
