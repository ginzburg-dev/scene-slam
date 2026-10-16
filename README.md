# SceneSLAM

**Walk through a room with a depth camera and get back two things: where the camera went, and what the room looks like.** Both in metres, both computed frame by frame, fast enough to keep up with the camera.

![SceneSLAM map of TUM RGB-D freiburg1_room](docs/images/fr1_room_map.jpg)

*An office reconstructed from 45 seconds of handheld Kinect footage. Orange is the path the camera took; the room is 1.4 million coloured points. Both were worked out by this program from nothing but the video and its depth channel.*

Every machine that moves on its own has to answer those two questions first — a robot that cannot locate itself cannot navigate, and a headset that cannot map the room cannot place anything in it. SceneSLAM is that answer, built from the geometry up rather than assembled from an existing SLAM library: features are matched between frames, depth lifts them into 3-D, and the camera motion that best explains the match is solved for directly. OpenCV provides the image primitives; the tracking, the mapping, and the pose arithmetic are written here in C++20, and each geometric step is pinned down by its own test.

| On TUM RGB-D `freiburg1_room`, 1359 frames | Result |
| --- | --- |
| Frames tracked without losing the camera | 1359 of 1359 |
| Time per frame | 29.9 ms — faster than the camera delivers them |
| Memory | 169 MB peak |
| Position error against motion capture | 0.412 m RMSE over the whole walk |
| Error accumulated per second of motion | 0.070 m, 3.34° |

Those last two lines are the ones worth reading. Second to second the camera is tracked to within seven centimetres, and that error never grows — but nothing cancels it either, so over a 45-second walk it accumulates into 41 cm of error on average and 78 cm at the worst point. Closing the loop when the camera revisits a place it has already seen is what would cancel it, and that is the part not written yet. [The full record](docs/results_tum_fr1_room.md) has the commands, the machine, and the trajectory plotted against ground truth.

![Keyframe frusta over the reconstructed office](docs/images/fr1_room_keyframes.jpg)

*The same reconstruction seen from inside, with every twentieth camera pose drawn as a frustum. Nothing here is modelled by hand or retouched.*

## What it does

- Reads TUM RGB-D sequences and pairs each colour frame with the depth frame closest in time.
- Tracks the camera by matching ORB features and solving PnP with RANSAC, in real metric units.
- Says `LOST` and explains why, rather than quietly emitting a wrong pose.
- Accumulates a bounded coloured voxel map and writes it as a PLY anyone can open.
- Writes a TUM trajectory, per-frame diagnostics, a run summary, and debug images.
- Produces bit-identical output run to run, and tests every geometric step.

Not implemented: loop closure, pose-graph optimisation, relocalisation, bundle adjustment. See [known limitations](docs/known_limitations.md) for the full list.

## Quick start

Requirements: CMake 3.20+, a C++20 compiler, OpenCV 4.x, and [uv](https://docs.astral.sh/uv/) for the viewer and evaluation tools.

```bash
make install                                   # uv sync: viewer and evaluation environment
make build OPENCV_DIR=/path/to/opencv4         # configure and compile
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

Accuracy is measured with `evo` against the sequence's ground truth. The script prints the exact commands, or runs them with `--execute`; alignment is rigid only, since RGB-D output is already in metres and correcting scale would hide a real error. Error windows default to 30 frames, one second at the TUM frame rate:

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

SceneSLAM implements its pipeline directly and uses OpenCV for image I/O, ORB, matching, PnP, and matrix operations. `evo` and Matplotlib are optional evaluation tools and are not runtime dependencies.

## Licence

MIT for the code. The images in `docs/images/` are renderings of SceneSLAM output computed from `rgbd_dataset_freiburg1_room` of the TUM RGB-D SLAM Dataset and Benchmark, which is licensed under Creative Commons Attribution 4.0; cite Sturm et al., IROS 2012 and follow the dataset licence when using TUM data.

