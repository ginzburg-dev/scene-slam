# SceneSLAM

**SceneSLAM is an RGB-D visual odometry and 3-D reconstruction system.** It takes the colour and depth frames of a consumer depth camera, estimates the camera trajectory in metric units, and reconstructs the scene as a coloured point cloud — the localisation and reconstruction layer that robot navigation, AR placement and 3-D scanning are built on.

![SceneSLAM map of TUM RGB-D freiburg1_room](docs/images/fr1_room_map.jpg)

*TUM RGB-D `freiburg1_room`: 45 seconds of handheld Kinect footage reconstructed as 1.4 M coloured points, with the estimated camera trajectory in orange.*

Written in C++20 with OpenCV, CPU-only, and faster than camera rate on a laptop. Feature tracking, PnP pose estimation, map integration and pose composition are implemented directly rather than wrapped around an existing SLAM library, and every geometric step has its own test. Loop closure and pose-graph optimisation are not implemented yet, so the map drifts over long trajectories — by how much is measured below.

## Results

Full `freiburg1_room` sequence, 1359 frames, i7-11800H, OpenCV 4.14:

| Metric | Result |
| --- | --- |
| Frames tracked / lost | 1359 / 0 |
| Tracking time per frame | 29.9 ms median, 34.8 ms p95 |
| Peak memory | 169 MB |
| ATE RMSE vs. motion capture | 0.412 m |
| RPE per second | 0.070 m, 3.34° |

Relative error stays at 7 cm per second of motion and does not grow; absolute error does, because nothing corrects it — 0.412 m RMSE and 0.78 m at worst over the 45-second loop. That gap is exactly what loop closure and pose-graph optimisation would remove, and it is the next thing to build. [Full record](docs/results_tum_fr1_room.md): commands, environment, and the trajectory plotted against ground truth.

![Keyframe frusta over the reconstructed office](docs/images/fr1_room_keyframes.jpg)

*The same reconstruction from inside the room, with every twentieth estimated pose drawn as a camera frustum.*

## Features

- TUM RGB-D input, with each colour frame paired to the nearest depth frame in time.
- Metric camera tracking: ORB matching, depth-backed 3-D points, PnP with RANSAC.
- Frames marked `LOST` with a stated reason instead of an unreliable pose.
- Bounded coloured voxel map, exported as ASCII PLY.
- TUM trajectory, per-frame JSONL diagnostics, run summary, and debug images.
- Deterministic output: bit-identical across runs, with tests on every geometric step.
- Interactive 3-D viewer for finished runs, and reproducible screenshot rendering.

Not implemented: loop closure, pose-graph optimisation, relocalisation, bundle adjustment, live camera input. See [known limitations](docs/known_limitations.md).

## Quick start

Requirements: CMake 3.20+, a C++20 compiler, OpenCV 4.x, and [uv](https://docs.astral.sh/uv/) for the viewer and evaluation tools.

```bash
make install                                   # uv sync: viewer and evaluation environment
make build OPENCV_DIR=/path/to/opencv/lib/cmake/opencv4   # configure and compile
make test                                      # full ctest suite
make run DATASET=/path/to/rgbd_dataset_freiburg1_room OUT=out/fr1_room
make view OUT=out/fr1_room                     # open the result in Rerun
make eval OUT=out/fr1_room GROUNDTRUTH=/path/to/groundtruth.txt
```

`make` on its own lists every target. Each step is a plain CMake or `uv` command, spelled out below for anyone without `make`.

## Build

The OpenCV modules used are `core`, `calib3d`, `features2d`, `imgcodecs`, and `imgproc`. Without `make`:

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

`--voxel-size` and `--max-points` size the map. The defaults (3 cm, 500 000 points) stop accepting new voxels partway through a room-sized sequence; `summary.json` reports `map_capacity_reached` so a truncated map is never silent. The images above use `--voxel-size 0.02 --max-points 4000000`.

Outputs are `trajectory_tum.txt`, `frames.jsonl`, `map.ply`, `summary.json`, and `debug/*.png`. Open `map.ply` in CloudCompare or inspect the whole run in Rerun.

## Interactive 3D viewer

Create the viewer environment with `uv` and open a real run:

```bash
uv sync
uv run python scripts/view_map.py out/fr1_xyz
```

The viewer opens the point cloud, the trajectory, and the keyframe frusta as separate layers that can be switched off individually, and the timeline replays the camera pose frame by frame. To look at the interface without a dataset, there is a clearly synthetic scene:

```bash
uv run python scripts/view_map.py --demo
uv run python scripts/view_map.py out/fr1_xyz --save out/fr1_xyz/recording.rrd
```

Open a saved recording later with `uv run rerun out/fr1_xyz/recording.rrd`. For reproducible screenshots, `--hide-panels`, `--hide-grid`, and `--eye/--look-at/--up` fix the composition, and the Rerun CLI renders it headlessly:

```bash
uv run python scripts/view_map.py out/fr1_xyz --save out/fr1_xyz/recording.rrd \
  --hide-panels --hide-grid --eye=5.1,-7.1,1.2 --look-at=0.3,0.0,0.9 --up=0,-1,0
uv run rerun out/fr1_xyz/recording.rrd --screenshot-to out/fr1_xyz/overview.png \
  --window-size 1600x1000
```

The world frame is the first camera frame, so `--up` is the scene's vertical expressed in that frame, not a fixed axis.

## Evaluation

Accuracy is measured with `evo` against the sequence's ground truth. The script prints the exact commands, or runs them with `--execute` when `evo` is installed; `make eval` fetches `evo` on the fly instead. Alignment is rigid only, since RGB-D output is already in metres and correcting scale would hide a real error. Error windows default to 30 frames, one second at the TUM frame rate:

```bash
python scripts/evaluate.py --groundtruth /path/to/groundtruth.txt \
  --trajectory out/fr1_xyz/trajectory_tum.txt
python scripts/evaluate.py --groundtruth /path/to/groundtruth.txt \
  --trajectory out/fr1_xyz/trajectory_tum.txt --execute
python scripts/render_trajectory.py out/fr1_xyz/trajectory_tum.txt \
  --out out/fr1_xyz/trajectory_topdown.png
```

This repository contains no dataset files. The one measured run it does claim is recorded in full in [results on TUM fr1/room](docs/results_tum_fr1_room.md); no result is claimed for any other sequence. See [benchmark protocol](docs/benchmark_protocol.md) and [known limitations](docs/known_limitations.md).

## Coordinate convention

`T_world_from_camera` maps camera-frame points into world coordinates. PnP estimates `T_current_from_reference`; poses compose as `T_world_from_current = T_world_from_reference * inverse(T_current_from_reference)`. TUM export is `timestamp tx ty tz qx qy qz qw`.

## Third-party components

SceneSLAM implements its pipeline directly and uses OpenCV for image I/O, ORB, matching, PnP, and matrix operations. Rerun powers the optional 3-D viewer, and `evo` and Matplotlib are optional evaluation tools; none of the three is a runtime dependency of `scene_slam` itself.

## Licence

MIT for the code. The images in `docs/images/` are renderings of SceneSLAM output computed from `rgbd_dataset_freiburg1_room` of the TUM RGB-D SLAM Dataset and Benchmark, which is licensed under Creative Commons Attribution 4.0; cite Sturm et al., IROS 2012 and follow the dataset licence when using TUM data.

