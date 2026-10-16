#!/usr/bin/env python3
"""Open SceneSLAM maps and trajectories in the Rerun data viewer."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class PoseSample:
    timestamp: float
    translation: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def read_ply_vertex_count(path: Path) -> int:
    if not path.is_file():
        raise FileNotFoundError(f"missing point cloud: {path}")
    with path.open("rb") as stream:
        if stream.readline().strip() != b"ply":
            raise ValueError(f"not a PLY file: {path}")
        vertex_count = None
        for raw_line in stream:
            line = raw_line.decode("ascii").strip()
            fields = line.split()
            if len(fields) == 3 and fields[:2] == ["element", "vertex"]:
                vertex_count = int(fields[2])
            if fields == ["end_header"]:
                break
        if vertex_count is None:
            raise ValueError(f"PLY header has no vertex count: {path}")
        return vertex_count


def read_tum_trajectory(path: Path) -> list[PoseSample]:
    if not path.is_file():
        raise FileNotFoundError(f"missing trajectory: {path}")
    poses = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 TUM fields")
        values = [float(field) for field in fields]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path}:{line_number}: non-finite pose value")
        quaternion = values[4:8]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-12:
            raise ValueError(f"{path}:{line_number}: zero quaternion")
        poses.append(
            PoseSample(
                values[0],
                (values[1], values[2], values[3]),
                tuple(value / norm for value in quaternion),
            )
        )
    if not poses:
        raise ValueError(f"trajectory contains no poses: {path}")
    return poses


def synthetic_scene() -> tuple[
    list[tuple[float, float, float]],
    list[tuple[int, int, int]],
    list[PoseSample],
]:
    points: list[tuple[float, float, float]] = []
    colors: list[tuple[int, int, int]] = []

    def surface(origin, axis_u, axis_v, count_u, count_v, color):
        for u in range(count_u):
            for v in range(count_v):
                fu = u / max(count_u - 1, 1)
                fv = v / max(count_v - 1, 1)
                points.append(
                    tuple(
                        origin[index] + axis_u[index] * fu + axis_v[index] * fv
                        for index in range(3)
                    )
                )
                colors.append(color)

    surface((-3.0, 0.0, -2.5), (6.0, 0.0, 0.0), (0.0, 0.0, 5.0), 70, 60, (51, 76, 107))
    surface((-3.0, 0.0, -2.5), (6.0, 0.0, 0.0), (0.0, 2.8, 0.0), 70, 35, (46, 122, 184))
    surface((-3.0, 0.0, -2.5), (0.0, 0.0, 5.0), (0.0, 2.8, 0.0), 60, 35, (107, 56, 173))
    surface((-1.1, 0.75, -0.7), (2.2, 0.0, 0.0), (0.0, 0.0, 1.4), 35, 24, (242, 122, 26))
    surface((-1.1, 0.0, -0.7), (0.0, 0.75, 0.0), (0.0, 0.0, 1.4), 12, 24, (158, 69, 20))

    poses = []
    for index in range(36):
        angle = 2.0 * math.pi * index / 36.0
        x = 2.1 * math.cos(angle)
        z = 1.7 * math.sin(angle)
        yaw = math.atan2(-x, -z)
        poses.append(
            PoseSample(
                float(index),
                (x, 1.25, z),
                (0.0, math.sin(yaw / 2.0), 0.0, math.cos(yaw / 2.0)),
            )
        )
    return points, colors, poses


def make_blueprint(rrb):
    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="/",
            name="SceneSLAM 3D",
            background=[5, 8, 16],
            line_grid=rrb.LineGrid3D(visible=True, spacing=1.0),
        ),
        rrb.SelectionPanel(state="expanded"),
        rrb.TimePanel(state="expanded", timeline="frame"),
        collapse_panels=False,
    )


def log_scene(args, poses, demo_data=None) -> int:
    try:
        import rerun as rr
        import rerun.blueprint as rrb
    except ImportError:
        print("Rerun dependency is missing. Install with: uv sync", file=sys.stderr)
        return 2

    blueprint = make_blueprint(rrb)
    if args.save is None:
        rr.init("scene_slam", spawn=True, default_blueprint=blueprint)
    else:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        rr.init("scene_slam")
        rr.save(args.save, default_blueprint=blueprint)

    if demo_data is None:
        rr.log("world", rr.ViewCoordinates.RDF, static=True)
        rr.log_file_from_path(
            args.result_dir / "map.ply",
            entity_path_prefix="world/map",
            static=True,
        )
    else:
        points, colors, _ = demo_data
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)
        rr.log(
            "world/map",
            rr.Points3D(
                points,
                colors=colors,
                radii=rr.Radius.ui_points(args.point_size),
            ),
            static=True,
        )

    trajectory = [pose.translation for pose in poses]
    rr.log(
        "world/trajectory",
        rr.LineStrips3D(
            [trajectory],
            colors=[255, 104, 24],
            radii=rr.Radius.ui_points(3.0),
        ),
        static=True,
    )

    for index, pose in enumerate(poses[:: args.frustum_step]):
        entity = f"world/keyframes/camera_{index:04d}"
        rr.log(
            entity,
            rr.Transform3D(
                translation=pose.translation,
                quaternion=rr.Quaternion(xyzw=pose.quaternion_xyzw),
            ),
            static=True,
        )
        rr.log(
            entity,
            rr.Pinhole(
                fov_y=1.0,
                aspect_ratio=4.0 / 3.0,
                camera_xyz=rr.ViewCoordinates.RDF,
                image_plane_distance=0.18,
                color=[55, 200, 255],
                line_width=0.006,
            ),
            static=True,
        )

    rr.log(
        "world/current_camera",
        rr.Pinhole(
            fov_y=1.0,
            aspect_ratio=4.0 / 3.0,
            camera_xyz=rr.ViewCoordinates.RDF,
            image_plane_distance=0.28,
            color=[255, 214, 64],
            line_width=0.01,
        ),
        static=True,
    )
    for frame_index, pose in enumerate(poses):
        rr.set_time("frame", sequence=frame_index)
        rr.set_time("timestamp", timestamp=pose.timestamp)
        rr.log(
            "world/current_camera",
            rr.Transform3D(
                translation=pose.translation,
                quaternion=rr.Quaternion(xyzw=pose.quaternion_xyzw),
            ),
        )

    rr.disconnect()
    if args.save is not None:
        print(f"saved Rerun recording: {args.save}")
    else:
        print("Rerun opened: use the timeline to scrub camera poses and the entity tree to toggle layers")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", nargs="?", type=Path)
    parser.add_argument("--demo", action="store_true", help="show a labelled synthetic scene")
    parser.add_argument("--summary-only", action="store_true", help="validate inputs without opening Rerun")
    parser.add_argument("--save", type=Path, help="write an .rrd recording instead of opening the viewer")
    parser.add_argument("--point-size", type=float, default=1.5, help="demo point radius in UI pixels")
    parser.add_argument("--frustum-step", type=int, default=15)
    args = parser.parse_args()
    if not args.demo and args.result_dir is None:
        parser.error("result_dir is required unless --demo is used")
    if args.point_size <= 0 or args.frustum_step <= 0:
        parser.error("point size and frustum step must be positive")
    return args


def main() -> int:
    args = parse_args()
    try:
        if args.demo:
            demo_data = synthetic_scene()
            points, _, poses = demo_data
            mode = "synthetic"
            point_count = len(points)
        else:
            demo_data = None
            point_count = read_ply_vertex_count(args.result_dir / "map.ply")
            poses = read_tum_trajectory(args.result_dir / "trajectory_tum.txt")
            mode = "result"
        print(f"mode={mode} points={point_count} poses={len(poses)}")
        if args.summary_only:
            return 0
        return log_scene(args, poses, demo_data)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
