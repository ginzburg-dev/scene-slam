#!/usr/bin/env python3
"""Interactive Open3D viewer for SceneSLAM maps and camera trajectories."""

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
    with path.open("r", encoding="ascii", errors="strict") as stream:
        if stream.readline().strip() != "ply":
            raise ValueError(f"not a PLY file: {path}")
        vertex_count = None
        for line in stream:
            fields = line.strip().split()
            if fields[:2] == ["format", "binary_little_endian"]:
                pass
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


def quaternion_xyzw_to_rotation(
    quaternion: tuple[float, float, float, float]
) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = quaternion
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def synthetic_scene() -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]], list[PoseSample]]:
    points: list[tuple[float, float, float]] = []
    colors: list[tuple[float, float, float]] = []

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

    surface((-3.0, 0.0, -2.5), (6.0, 0.0, 0.0), (0.0, 0.0, 5.0), 70, 60, (0.20, 0.30, 0.42))
    surface((-3.0, 0.0, -2.5), (6.0, 0.0, 0.0), (0.0, 2.8, 0.0), 70, 35, (0.18, 0.48, 0.72))
    surface((-3.0, 0.0, -2.5), (0.0, 0.0, 5.0), (0.0, 2.8, 0.0), 60, 35, (0.42, 0.22, 0.68))
    surface((-1.1, 0.75, -0.7), (2.2, 0.0, 0.0), (0.0, 0.0, 1.4), 35, 24, (0.95, 0.48, 0.10))
    surface((-1.1, 0.0, -0.7), (0.0, 0.75, 0.0), (0.0, 0.0, 1.4), 12, 24, (0.62, 0.27, 0.08))

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


def transform_local(point, pose: PoseSample):
    rotation = quaternion_xyzw_to_rotation(pose.quaternion_xyzw)
    return tuple(
        sum(rotation[row][column] * point[column] for column in range(3))
        + pose.translation[row]
        for row in range(3)
    )


def make_line_set(o3d, np, points, lines, color):
    geometry = o3d.geometry.LineSet()
    geometry.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    geometry.lines = o3d.utility.Vector2iVector(
        np.asarray(lines, dtype=int).reshape((-1, 2))
    )
    geometry.colors = o3d.utility.Vector3dVector(
        np.tile(np.asarray(color, dtype=float), (len(lines), 1))
    )
    return geometry


def trajectory_geometry(o3d, np, poses):
    points = [pose.translation for pose in poses]
    lines = [(index, index + 1) for index in range(len(points) - 1)]
    return make_line_set(o3d, np, points, lines, (1.0, 0.42, 0.05))


def frusta_geometry(o3d, np, poses, step):
    local = [(0.0, 0.0, 0.0), (-0.12, -0.08, 0.22), (0.12, -0.08, 0.22),
             (0.12, 0.08, 0.22), (-0.12, 0.08, 0.22)]
    edges = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]
    points = []
    lines = []
    for pose in poses[::step]:
        base = len(points)
        points.extend(transform_local(point, pose) for point in local)
        lines.extend((base + start, base + end) for start, end in edges)
    return make_line_set(o3d, np, points, lines, (0.10, 0.78, 1.0))


def grid_geometry(o3d, np):
    points = []
    lines = []
    for value in range(-6, 7):
        base = len(points)
        points.extend([(value, 0.0, -6.0), (value, 0.0, 6.0),
                       (-6.0, 0.0, value), (6.0, 0.0, value)])
        lines.extend([(base, base + 1), (base + 2, base + 3)])
    return make_line_set(o3d, np, points, lines, (0.12, 0.16, 0.22))


def run_viewer(args, poses, demo_data=None) -> int:
    try:
        import numpy as np
        import open3d as o3d
    except ImportError:
        print(
            "Open3D viewer dependency is missing. Install with: uv sync",
            file=sys.stderr,
        )
        return 2

    if demo_data is None:
        cloud = o3d.io.read_point_cloud(str(args.result_dir / "map.ply"))
        if cloud.is_empty():
            raise ValueError("point cloud is empty")
    else:
        demo_points, demo_colors, _ = demo_data
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.asarray(demo_points, dtype=float))
        cloud.colors = o3d.utility.Vector3dVector(np.asarray(demo_colors, dtype=float))

    layers = {
        "map": cloud,
        "trajectory": trajectory_geometry(o3d, np, poses),
        "cameras": frusta_geometry(o3d, np, poses, max(1, args.frustum_step)),
        "grid": grid_geometry(o3d, np),
        "axes": o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5),
    }
    visible = {name: True for name in layers}
    visualizer = o3d.visualization.VisualizerWithKeyCallback()
    visualizer.create_window("SceneSLAM — RGB-D Map Viewer", 1600, 900)
    for index, geometry in enumerate(layers.values()):
        visualizer.add_geometry(geometry, reset_bounding_box=index == 0)

    def toggle(name):
        def callback(window):
            if visible[name]:
                window.remove_geometry(layers[name], reset_bounding_box=False)
            else:
                window.add_geometry(layers[name], reset_bounding_box=False)
            visible[name] = not visible[name]
            return False
        return callback

    screenshot_path = args.screenshot or Path("sceneslam_view.png")

    def capture(window):
        screenshot_path.parent.mkdir(parents=True, exist_ok=True)
        window.capture_screen_image(str(screenshot_path), do_render=True)
        print(f"saved screenshot: {screenshot_path}")
        return False

    visualizer.register_key_callback(ord("1"), toggle("map"))
    visualizer.register_key_callback(ord("2"), toggle("trajectory"))
    visualizer.register_key_callback(ord("3"), toggle("cameras"))
    visualizer.register_key_callback(ord("4"), toggle("grid"))
    visualizer.register_key_callback(ord("S"), capture)
    options = visualizer.get_render_option()
    options.background_color = np.asarray((0.008, 0.012, 0.025))
    options.point_size = args.point_size
    options.line_width = 2.0
    options.show_coordinate_frame = True
    view = visualizer.get_view_control()
    view.set_lookat((0.0, 1.1, 0.0))
    view.set_front((0.55, 0.28, -0.79))
    view.set_up((0.0, 1.0, 0.0))
    view.set_zoom(0.62)
    print("Controls: mouse=orbit/pan/zoom, 1=map, 2=trajectory, 3=cameras, 4=grid, S=screenshot, Q=quit")

    if args.screenshot:
        visualizer.poll_events()
        visualizer.update_renderer()
        rendered = np.asarray(
            visualizer.capture_screen_float_buffer(do_render=True), dtype=float
        )
        spatial_range = np.ptp(rendered, axis=(0, 1)) if rendered.size else []
        if rendered.size == 0 or max(spatial_range, default=0.0) < 0.01:
            visualizer.destroy_window()
            raise RuntimeError("rendered screenshot is blank")
        capture(visualizer)
    else:
        visualizer.run()
    visualizer.destroy_window()
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", nargs="?", type=Path)
    parser.add_argument("--demo", action="store_true", help="show a synthetic room without a dataset")
    parser.add_argument("--summary-only", action="store_true", help="validate inputs without opening Open3D")
    parser.add_argument("--screenshot", type=Path, help="capture one frame and exit")
    parser.add_argument("--point-size", type=float, default=2.2)
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
        return run_viewer(args, poses, demo_data)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

