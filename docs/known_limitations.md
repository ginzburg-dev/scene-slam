# Known limitations

- This is RGB-D visual odometry and accumulated mapping, not full SLAM.
- There is no loop closure, global relocalisation, pose graph, or bundle adjustment.
- Tracking uses frame-to-reference ORB/PnP and limited recovery from the last reliable frame.
- Depth must already be registered to RGB and use the configured metric scale.
- Dynamic objects, rolling shutter, exposure changes, repeated textures, and low-texture scenes are not modelled explicitly.
- The voxel map keeps the first point in each voxel; it is not a TSDF or watertight mesh.
- Runtime is offline CPU processing; real-time camera, ROS 2, and hardware claims have not been validated.
- No ATE, RPE, FPS, or accuracy result is claimed until a real dataset run is recorded.
