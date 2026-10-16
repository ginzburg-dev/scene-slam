#pragma once

#include <opencv2/core/types.hpp>

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace sceneslam {

struct CameraModel {
  double fx{};
  double fy{};
  double cx{};
  double cy{};
  int width{};
  int height{};
  double depth_units_per_metre{};
  double min_depth_m{0.1};
  double max_depth_m{8.0};

  void validate() const {
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0 ||
        !std::isfinite(cx) || !std::isfinite(cy) || width <= 0 || height <= 0 ||
        !std::isfinite(depth_units_per_metre) || depth_units_per_metre <= 0.0 ||
        !std::isfinite(min_depth_m) || !std::isfinite(max_depth_m) ||
        min_depth_m <= 0.0 || max_depth_m <= min_depth_m) {
      throw std::invalid_argument("invalid camera calibration");
    }
  }

  [[nodiscard]] std::optional<cv::Point3d> backProjectIfValid(
      const cv::Point2d& pixel, std::uint16_t depth_raw) const {
    if (depth_raw == 0) {
      return std::nullopt;
    }
    const double z = static_cast<double>(depth_raw) / depth_units_per_metre;
    if (!std::isfinite(z) || z < min_depth_m || z > max_depth_m) {
      return std::nullopt;
    }
    return cv::Point3d{(pixel.x - cx) * z / fx, (pixel.y - cy) * z / fy, z};
  }

  [[nodiscard]] cv::Point3d backProject(const cv::Point2d& pixel,
                                        std::uint16_t depth_raw) const {
    const auto point = backProjectIfValid(pixel, depth_raw);
    if (!point) {
      throw std::invalid_argument("invalid depth sample");
    }
    return *point;
  }

  [[nodiscard]] cv::Point2d project(const cv::Point3d& point) const {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || point.z <= 0.0) {
      throw std::invalid_argument("point cannot be projected");
    }
    return {fx * point.x / point.z + cx, fy * point.y / point.z + cy};
  }
};

}  // namespace sceneslam

