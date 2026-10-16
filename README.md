# SceneSLAM

CPU-first C++20 RGB-D visual odometry and bounded coloured point-cloud mapping.

**Current status:** RGB-D odometry & mapping; SLAM features are in progress. Loop closure and pose-graph optimisation are not implemented.

## Features

- TUM RGB-D list parsing with nearest-timestamp association.
- ORB matching and metric 3D-to-2D PnP-RANSAC tracking.
- Explicit camera-to-world pose convention and `LOST` frames.
- Bounded voxel map exported as coloured ASCII PLY.
- TUM trajectory, JSONL frame diagnostics, summary JSON, and debug images.
- Deterministic geometry, association, PnP, output, pipeline, and CLI tests.

## Build

Requirements: CMake 3.20+, a C++20 compiler, and OpenCV 4.x with `core`, `calib3d`, `features2d`, `imgcodecs`, and `imgproc`.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

On Windows with the official OpenCV archive, `OpenCV_DIR` is normally its `build` directory. Ensure the matching OpenCV DLL directory is in `PATH` when running the executable directly.

## Run

No dataset is downloaded automatically. Supply an extracted TUM RGB-D sequence:

```bash
scene_slam selftest
scene_slam run --dataset /path/to/rgbd_dataset_freiburg1_xyz \
  --camera configs/tum_freiburg1.yaml --out out/fr1_xyz --max-frames 250
```

Outputs are `trajectory_tum.txt`, `frames.jsonl`, `map.ply`, `summary.json`, and `debug/*.png`. Open `map.ply` in CloudCompare, MeshLab, or Open3D.

## Interactive 3D viewer

Create the viewer environment with `uv` and open a real run:

```bash
uv sync
uv run python scripts/view_map.py out/fr1_xyz
```

Keyboard controls: `1` map, `2` trajectory, `3` camera frusta, `4` ground grid, `S` screenshot, `Q` close. The mouse orbits, pans, and zooms. To preview the interface without a dataset, use an explicitly synthetic scene:

```bash
uv run python scripts/view_map.py --demo
uv run python scripts/view_map.py out/fr1_xyz --screenshot out/fr1_xyz/view.png
```

## Evaluation

Install `evo` separately, then print or execute metric-scale commands:

```bash
python scripts/evaluate.py --groundtruth /path/to/groundtruth.txt \
  --trajectory out/fr1_xyz/trajectory_tum.txt
python scripts/evaluate.py --groundtruth /path/to/groundtruth.txt \
  --trajectory out/fr1_xyz/trajectory_tum.txt --execute
python scripts/render_trajectory.py out/fr1_xyz/trajectory_tum.txt \
  --out out/fr1_xyz/trajectory_topdown.png
```

This repository does not contain datasets or measured benchmark claims. See [benchmark protocol](docs/benchmark_protocol.md) and [known limitations](docs/known_limitations.md).

## Coordinate convention

`T_world_from_camera` maps camera-frame points into world coordinates. PnP estimates `T_current_from_reference`; poses compose as `T_world_from_current = T_world_from_reference * inverse(T_current_from_reference)`. TUM export is `timestamp tx ty tz qx qy qz qw`.

## Third-party components

SceneSLAM implements its pipeline directly and uses OpenCV for image I/O, ORB, matching, PnP, and matrix operations. `evo` and Matplotlib are optional evaluation tools and are not runtime dependencies.

## Licence

MIT. TUM RGB-D data has separate terms; cite Sturm et al., IROS 2012 and follow the dataset licence.

