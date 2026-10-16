#!/usr/bin/env python3
"""Validate a TUM trajectory and optionally render its X-Z top-down path."""

import argparse
from pathlib import Path


def read_positions(path: Path) -> list[tuple[float, float, float]]:
    positions = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ValueError(f"line {line_number}: expected 8 TUM fields")
        values = [float(field) for field in fields]
        positions.append((values[1], values[2], values[3]))
    if not positions:
        raise ValueError("trajectory contains no poses")
    return positions


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--out", type=Path, default=Path("trajectory_topdown.png"))
    parser.add_argument("--summary-only", action="store_true")
    args = parser.parse_args()

    positions = read_positions(args.trajectory)
    xs = [point[0] for point in positions]
    zs = [point[2] for point in positions]
    print(
        f"poses={len(positions)} x_range_m={max(xs) - min(xs):g} "
        f"z_range_m={max(zs) - min(zs):g}"
    )
    if args.summary_only:
        return 0

    import matplotlib.pyplot as plt

    _, axis = plt.subplots(figsize=(7, 6))
    axis.plot(xs, zs, linewidth=1.5)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("world X [m]")
    axis.set_ylabel("world Z [m]")
    axis.set_title("SceneSLAM camera trajectory")
    axis.grid(True, alpha=0.3)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(args.out, dpi=160)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

