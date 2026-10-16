# Changelog

All notable changes to this project are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[semantic versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-10-17

First release: RGB-D visual odometry and coloured 3-D reconstruction, measured
end to end on a public dataset.

### Added

- Camera and pose geometry with an explicit `T_world_from_camera` convention.
- TUM RGB-D parsing that pairs each colour frame with the nearest depth frame
  in time and reports the frames it could not pair.
- Metric camera tracking: ORB matching, depth-backed 3-D points, and PnP with
  RANSAC, with frames marked `LOST` and a stated reason.
- Bounded coloured voxel map, exported as ASCII PLY, with `--voxel-size` and
  `--max-points` to size it and `map_capacity_reached` in the run summary.
- Run outputs: TUM trajectory, per-frame JSONL diagnostics, summary JSON, and
  debug images.
- `scene_slam` command line with `run`, `selftest`, and `--version`.
- Interactive 3-D viewer built on Rerun, with a synthetic demo scene and fixed
  camera, panel, and grid options for reproducible screenshots.
- `scripts/evaluate.py` for ATE and RPE against ground truth with rigid
  alignment, and `scripts/render_trajectory.py` for a top-down plot.
- Makefile covering install, build, test, run, view, screenshot, and evaluate.
- Measured run on TUM RGB-D `freiburg1_room`: 1359 of 1359 frames tracked,
  29.9 ms median per frame, 169 MB peak, 0.412 m ATE RMSE, 0.070 m and 3.34°
  per second of motion. Recorded in `docs/results_tum_fr1_room.md`.
- Test suite covering geometry, frame association, PnP, outputs, the pipeline,
  the CLI, and the Python scripts.

[0.1.0]: https://github.com/ginzburg-dev/scene-slam/releases/tag/v0.1.0
