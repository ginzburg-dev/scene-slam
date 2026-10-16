# Architecture

The input layer parses `rgb.txt` and `depth.txt`, associates each RGB timestamp with the nearest unused depth timestamp inside the configured skew, then validates image type and calibration size.

For the first valid frame, the world pose is identity. Later frames are matched against the last reliable reference using ORB and a Hamming ratio test. Valid reference depths become metric 3D points; their current keypoints become 2D observations. PnP-RANSAC estimates `T_current_from_reference` and quality gates reject weak geometry, high error, or implausible motion.

Accepted poses use:

```text
T_world_from_current = T_world_from_reference * inverse(T_current_from_reference)
```

Only accepted poses enter the TUM trajectory and map. Every attempted frame enters `frames.jsonl`; failed frames are `LOST` with a reason. The last reliable reference remains available for limited recovery.

The map backprojects sampled RGB-D pixels, transforms them into world coordinates, stores one coloured point per voxel, and stops admitting new voxels at the configured point cap.

