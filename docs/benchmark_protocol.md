# Benchmark protocol

1. Use an unmodified TUM RGB-D sequence and record its exact name.
2. Record OS, CPU, compiler, OpenCV version, commit, and complete run command.
3. Verify calibration and `depth_units_per_metre` before evaluating.
4. Run the same frame range and settings for every comparison.
5. Evaluate metric trajectories with rigid alignment only; never use scale correction to hide RGB-D scale errors.
6. Report ATE RMSE, RPE translation/rotation, tracked/lost counts, median and p95 tracking time, map points, and memory.
7. Preserve failure intervals and avoid reporting a metric when ground truth or timestamp association is invalid.

Suggested first command:

```bash
scene_slam run --dataset /data/rgbd_dataset_freiburg1_xyz \
  --camera configs/tum_freiburg1.yaml --out out/fr1_xyz
```

Then use `scripts/evaluate.py` with the sequence `groundtruth.txt` and generated trajectory.

[Results on TUM fr1/room](results_tum_fr1_room.md) is a worked example of this protocol.

