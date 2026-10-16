#pragma once

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

#include <array>

namespace sceneslam {

using Pose = cv::Matx44d;

[[nodiscard]] Pose identityPose();
[[nodiscard]] Pose poseFromRotationTranslation(const cv::Matx33d& rotation,
                                               const cv::Vec3d& translation);
[[nodiscard]] Pose invertPose(const Pose& pose);
[[nodiscard]] Pose composePose(const Pose& lhs, const Pose& rhs);
[[nodiscard]] cv::Point3d transformPoint(const Pose& pose,
                                         const cv::Point3d& point);
[[nodiscard]] std::array<double, 4> rotationToQuaternionXyzw(const Pose& pose);

}  // namespace sceneslam

