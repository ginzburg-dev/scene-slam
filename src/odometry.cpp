#include "sceneslam/odometry.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>

namespace sceneslam {
namespace {

cv::Matx33d cameraMatrix(const CameraModel& camera) {
  return {camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0};
}

TrackingResult lost(std::string reason) {
  TrackingResult result;
  result.reason = std::move(reason);
  return result;
}

std::optional<double> bilinearDepthMetres(const cv::Mat& depth,
                                          const cv::Point2f& pixel,
                                          const CameraModel& camera) {
  if (pixel.x < 0.0F || pixel.y < 0.0F || pixel.x >= depth.cols - 1.0F ||
      pixel.y >= depth.rows - 1.0F) {
    return std::nullopt;
  }
  const int x0 = static_cast<int>(std::floor(pixel.x));
  const int y0 = static_cast<int>(std::floor(pixel.y));
  const double dx = pixel.x - x0;
  const double dy = pixel.y - y0;
  const std::uint16_t samples[4] = {
      depth.at<std::uint16_t>(y0, x0), depth.at<std::uint16_t>(y0, x0 + 1),
      depth.at<std::uint16_t>(y0 + 1, x0),
      depth.at<std::uint16_t>(y0 + 1, x0 + 1)};
  for (const auto raw : samples) {
    const double metres = static_cast<double>(raw) / camera.depth_units_per_metre;
    if (raw == 0 || metres < camera.min_depth_m || metres > camera.max_depth_m) {
      return std::nullopt;
    }
  }
  const double top = samples[0] * (1.0 - dx) + samples[1] * dx;
  const double bottom = samples[2] * (1.0 - dx) + samples[3] * dx;
  return (top * (1.0 - dy) + bottom * dy) / camera.depth_units_per_metre;
}

}  // namespace

TrackingResult estimatePosePnP(const std::vector<cv::Point3d>& object_points,
                               const std::vector<cv::Point2d>& image_points,
                               const CameraModel& camera,
                               const OdometryConfig& config) {
  camera.validate();
  if (object_points.size() != image_points.size()) {
    throw std::invalid_argument("3D and 2D correspondence counts differ");
  }
  if (object_points.size() < static_cast<std::size_t>(config.min_correspondences)) {
    return lost("insufficient 3D-2D correspondences");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto& point : image_points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  if (max_x - min_x < config.min_image_extent_px ||
      max_y - min_y < config.min_image_extent_px) {
    return lost("correspondences have poor image coverage");
  }

  cv::Mat rotation_vector;
  cv::Mat translation_vector;
  cv::Mat inlier_indices;
  const bool solved = cv::solvePnPRansac(
      object_points, image_points, cv::Mat(cameraMatrix(camera)), cv::noArray(),
      rotation_vector, translation_vector, false, config.ransac_iterations,
      static_cast<float>(config.ransac_reprojection_error_px),
      config.ransac_confidence, inlier_indices, cv::SOLVEPNP_EPNP);
  if (!solved || inlier_indices.rows < config.min_inliers) {
    return lost("PnP RANSAC found too few inliers");
  }

  std::vector<cv::Point3d> inlier_objects;
  std::vector<cv::Point2d> inlier_images;
  inlier_objects.reserve(inlier_indices.rows);
  inlier_images.reserve(inlier_indices.rows);
  for (int row = 0; row < inlier_indices.rows; ++row) {
    const int index = inlier_indices.at<int>(row, 0);
    inlier_objects.push_back(object_points[static_cast<std::size_t>(index)]);
    inlier_images.push_back(image_points[static_cast<std::size_t>(index)]);
  }
  cv::solvePnPRefineLM(inlier_objects, inlier_images, cv::Mat(cameraMatrix(camera)),
                       cv::noArray(), rotation_vector, translation_vector);

  std::vector<cv::Point2d> projected;
  cv::projectPoints(inlier_objects, rotation_vector, translation_vector,
                    cv::Mat(cameraMatrix(camera)), cv::noArray(), projected);
  double mean_error = 0.0;
  for (std::size_t index = 0; index < projected.size(); ++index) {
    mean_error += cv::norm(projected[index] - inlier_images[index]);
  }
  mean_error /= static_cast<double>(projected.size());
  if (!std::isfinite(mean_error) ||
      mean_error > config.max_mean_reprojection_error_px) {
    return lost("PnP reprojection error exceeds limit");
  }

  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix);
  cv::Matx33d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = rotation_matrix.at<double>(row, col);
    }
  }
  const cv::Vec3d translation{translation_vector.at<double>(0),
                              translation_vector.at<double>(1),
                              translation_vector.at<double>(2)};
  const double translation_norm = cv::norm(translation);
  const double cosine = std::clamp((cv::trace(cv::Mat(rotation))[0] - 1.0) * 0.5,
                                   -1.0, 1.0);
  const double rotation_degrees = std::acos(cosine) * 180.0 / std::acos(-1.0);
  if (translation_norm > config.max_translation_m ||
      rotation_degrees > config.max_rotation_deg) {
    return lost("estimated camera motion exceeds limit");
  }

  TrackingResult result;
  result.status = TrackingStatus::Tracking;
  result.T_current_from_reference =
      poseFromRotationTranslation(rotation, translation);
  result.pnp_inliers = inlier_indices.rows;
  result.reprojection_error_px = mean_error;
  result.relative_translation_m = translation_norm;
  result.relative_rotation_deg = rotation_degrees;
  return result;
}

RgbdOdometry::RgbdOdometry(CameraModel camera, OdometryConfig config)
    : camera_(std::move(camera)), config_(std::move(config)) {
  camera_.validate();
}

TrackingResult RgbdOdometry::track(const RgbdFrame& reference,
                                   const RgbdFrame& current) const {
  if (reference.color.empty() || current.color.empty() || reference.depth.empty() ||
      reference.depth.type() != CV_16UC1) {
    return lost("invalid RGB-D frame for tracking");
  }

  cv::Mat reference_gray;
  cv::Mat current_gray;
  cv::cvtColor(reference.color, reference_gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(current.color, current_gray, cv::COLOR_BGR2GRAY);
  auto orb = cv::ORB::create(config_.orb_features);
  std::vector<cv::KeyPoint> reference_keypoints;
  std::vector<cv::KeyPoint> current_keypoints;
  cv::Mat reference_descriptors;
  cv::Mat current_descriptors;
  orb->detectAndCompute(reference_gray, cv::noArray(), reference_keypoints,
                        reference_descriptors);
  orb->detectAndCompute(current_gray, cv::noArray(), current_keypoints,
                        current_descriptors);
  if (reference_descriptors.empty() || current_descriptors.empty()) {
    TrackingResult result = lost("ORB found no usable descriptors");
    result.reference_features = static_cast<int>(reference_keypoints.size());
    result.current_features = static_cast<int>(current_keypoints.size());
    return result;
  }

  cv::BFMatcher matcher(cv::NORM_HAMMING, false);
  std::vector<std::vector<cv::DMatch>> neighbors;
  matcher.knnMatch(reference_descriptors, current_descriptors, neighbors, 2);
  std::vector<cv::DMatch> matches;
  std::set<int> used_current;
  for (const auto& candidate : neighbors) {
    if (candidate.size() < 2 ||
        candidate[0].distance >= config_.ratio_test * candidate[1].distance ||
        !used_current.insert(candidate[0].trainIdx).second) {
      continue;
    }
    matches.push_back(candidate[0]);
  }

  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;
  for (const auto& match : matches) {
    const cv::Point2f pixel = reference_keypoints[match.queryIdx].pt;
    const auto depth_m = bilinearDepthMetres(reference.depth, pixel, camera_);
    if (!depth_m) {
      continue;
    }
    object_points.emplace_back((pixel.x - camera_.cx) * *depth_m / camera_.fx,
                               (pixel.y - camera_.cy) * *depth_m / camera_.fy,
                               *depth_m);
    image_points.emplace_back(current_keypoints[match.trainIdx].pt);
  }

  TrackingResult result = estimatePosePnP(object_points, image_points, camera_, config_);
  result.reference_features = static_cast<int>(reference_keypoints.size());
  result.current_features = static_cast<int>(current_keypoints.size());
  result.matches = static_cast<int>(matches.size());
  result.valid_depth_features = static_cast<int>(object_points.size());
  return result;
}

}  // namespace sceneslam
