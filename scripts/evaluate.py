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
    parser.add_argument(
        "--delta-frames",
        type=int,
        default=30,
        help="RPE window in frames; 30 is one second at the TUM 30 Hz rate",
    )
    args = parser.parse_args()
    if args.delta_frames <= 0:
        parser.error("--delta-frames must be positive")

    delta = ["--delta", str(args.delta_frames), "--delta_unit", "f"]
    commands = [
        ["evo_ape", "tum", args.groundtruth, args.trajectory, "--align"],
        ["evo_rpe", "tum", args.groundtruth, args.trajectory, "--align", *delta],
        [
            "evo_rpe",
            "tum",
            args.groundtruth,
            args.trajectory,
            "--align",
            *delta,
            "--pose_relation",
            "angle_deg",
        ],
    ]
    for command in commands:
        print(shlex.join(command))
        if args.execute:
            subprocess.run(command, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

