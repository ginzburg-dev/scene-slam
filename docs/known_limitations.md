# Known limitations

- This is RGB-D visual odometry and accumulated mapping, not full SLAM.
- There is no loop closure, global relocalisation, pose graph, or bundle adjustment.
- Tracking uses frame-to-reference ORB/PnP and limited recovery from the last reliable frame.
- Depth must already be registered to RGB and use the configured metric scale.
- Dynamic objects, rolling shutter, exposure changes, repeated textures, and low-texture scenes are not modelled explicitly.
- The voxel map keeps the first point in each voxel; it is not a TSDF or watertight mesh.
- Runtime is offline CPU processing; real-time camera, ROS 2, and hardware claims have not been validated.
- The Rerun viewer visualises completed runs; live sensor capture is not implemented yet.
- The only accuracy, timing, and memory results claimed are from one recorded run on TUM RGB-D `freiburg1_room`; see [results](results_tum_fr1_room.md). Nothing is claimed for any other sequence, scene type, or machine.
- That run drifts to 0.41 m ATE RMSE over 45 seconds while its per-second relative error stays near 0.07 m, which is the accumulated drift that loop closure would bound.
