#!/usr/bin/env python3
"""Print or execute reproducible evo commands for metric RGB-D trajectories."""

import argparse
import shlex
import subprocess


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--groundtruth", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    commands = [
        ["evo_ape", "tum", args.groundtruth, args.trajectory, "--align"],
        ["evo_rpe", "tum", args.groundtruth, args.trajectory, "--align"],
    ]
    for command in commands:
        print(shlex.join(command))
        if args.execute:
            subprocess.run(command, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

