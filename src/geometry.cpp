#include "sceneslam/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace sceneslam {

Pose identityPose() { return Pose::eye(); }

Pose poseFromRotationTranslation(const cv::Matx33d& rotation,
                                 const cv::Vec3d& translation) {
  Pose pose = Pose::eye();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      pose(row, col) = rotation(row, col);
    }
    pose(row, 3) = translation[row];
  }
  return pose;
}

Pose invertPose(const Pose& pose) {
  cv::Matx33d rotation;
  cv::Vec3d translation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = pose(row, col);
    }
    translation[row] = pose(row, 3);
  }
  const cv::Matx33d inverse_rotation = rotation.t();
  return poseFromRotationTranslation(inverse_rotation,
                                     -(inverse_rotation * translation));
}

Pose composePose(const Pose& lhs, const Pose& rhs) { return lhs * rhs; }

cv::Point3d transformPoint(const Pose& pose, const cv::Point3d& point) {
  const cv::Vec4d homogeneous{point.x, point.y, point.z, 1.0};
  const cv::Vec4d transformed = pose * homogeneous;
  return {transformed[0], transformed[1], transformed[2]};
}

std::array<double, 4> rotationToQuaternionXyzw(const Pose& pose) {
  const double trace = pose(0, 0) + pose(1, 1) + pose(2, 2);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    w = 0.25 * scale;
    x = (pose(2, 1) - pose(1, 2)) / scale;
    y = (pose(0, 2) - pose(2, 0)) / scale;
    z = (pose(1, 0) - pose(0, 1)) / scale;
  } else if (pose(0, 0) > pose(1, 1) && pose(0, 0) > pose(2, 2)) {
    const double scale = 2.0 * std::sqrt(1.0 + pose(0, 0) - pose(1, 1) - pose(2, 2));
    w = (pose(2, 1) - pose(1, 2)) / scale;
    x = 0.25 * scale;
    y = (pose(0, 1) + pose(1, 0)) / scale;
    z = (pose(0, 2) + pose(2, 0)) / scale;
  } else if (pose(1, 1) > pose(2, 2)) {
    const double scale = 2.0 * std::sqrt(1.0 + pose(1, 1) - pose(0, 0) - pose(2, 2));
    w = (pose(0, 2) - pose(2, 0)) / scale;
    x = (pose(0, 1) + pose(1, 0)) / scale;
    y = 0.25 * scale;
    z = (pose(1, 2) + pose(2, 1)) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + pose(2, 2) - pose(0, 0) - pose(1, 1));
    w = (pose(1, 0) - pose(0, 1)) / scale;
    x = (pose(0, 2) + pose(2, 0)) / scale;
    y = (pose(1, 2) + pose(2, 1)) / scale;
    z = 0.25 * scale;
  }

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  return {x / norm, y / norm, z / norm, w / norm};
}

}  // namespace sceneslam
