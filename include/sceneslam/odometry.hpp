#pragma once

#include "sceneslam/camera.hpp"
#include "sceneslam/geometry.hpp"
#include "sceneslam/rgbd.hpp"

#include <opencv2/core/types.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace sceneslam {

enum class TrackingStatus { Tracking, Lost };

struct OdometryConfig {
  int orb_features{1500};
  double ratio_test{0.75};
  int min_correspondences{12};
  int min_inliers{10};
  double min_image_extent_px{30.0};
  int ransac_iterations{200};
  double ransac_reprojection_error_px{3.0};
  double ransac_confidence{0.999};
  double max_mean_reprojection_error_px{2.5};
  double max_translation_m{1.5};
  double max_rotation_deg{60.0};
};

struct TrackingResult {
  TrackingStatus status{TrackingStatus::Lost};
  Pose T_current_from_reference{identityPose()};
  int reference_features{};
  int current_features{};
  int matches{};
  int valid_depth_features{};
  int pnp_inliers{};
  double reprojection_error_px{};
  double relative_translation_m{};
  double relative_rotation_deg{};
  std::string reason;
};

[[nodiscard]] TrackingResult estimatePosePnP(
    const std::vector<cv::Point3d>& object_points,
    const std::vector<cv::Point2d>& image_points, const CameraModel& camera,
    const OdometryConfig& config = {});

class RgbdOdometry {
 public:
  explicit RgbdOdometry(CameraModel camera, OdometryConfig config = {});
  [[nodiscard]] TrackingResult track(const RgbdFrame& reference,
                                     const RgbdFrame& current) const;

 private:
  CameraModel camera_;
  OdometryConfig config_;
};

}  // namespace sceneslam

