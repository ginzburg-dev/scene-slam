#pragma once

#include "sceneslam/camera.hpp"

#include <opencv2/core/mat.hpp>

#include <cstddef>
#include <filesystem>
#include <vector>

namespace sceneslam {

struct TimestampedPath {
  double timestamp{};
  std::filesystem::path path;
};

struct FramePair {
  TimestampedPath rgb;
  TimestampedPath depth;
};

struct AssociationResult {
  std::vector<FramePair> pairs;
  std::size_t unmatched_rgb{};
  std::size_t unmatched_depth{};
};

struct RgbdFrame {
  double timestamp{};
  double depth_timestamp{};
  cv::Mat color;
  cv::Mat depth;
  std::filesystem::path color_path;
  std::filesystem::path depth_path;
};

[[nodiscard]] std::vector<TimestampedPath> readTumList(
    const std::filesystem::path& list_path,
    const std::filesystem::path& dataset_root);
[[nodiscard]] AssociationResult associateNearest(
    const std::vector<TimestampedPath>& rgb,
    const std::vector<TimestampedPath>& depth, double max_skew_seconds);
[[nodiscard]] RgbdFrame loadRgbdFrame(const FramePair& pair,
                                      const CameraModel& camera);

}  // namespace sceneslam

