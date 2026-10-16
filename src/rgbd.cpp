#include "sceneslam/rgbd.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sceneslam {

std::vector<TimestampedPath> readTumList(
    const std::filesystem::path& list_path,
    const std::filesystem::path& dataset_root) {
  std::ifstream stream(list_path);
  if (!stream) {
    throw std::runtime_error("cannot open TUM list: " + list_path.string());
  }

  std::vector<TimestampedPath> entries;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(stream, line)) {
    ++line_number;
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::istringstream row(line);
    double timestamp = 0.0;
    std::string relative_path;
    if (!(row >> timestamp >> relative_path) || !std::isfinite(timestamp) ||
        relative_path.empty()) {
      throw std::runtime_error("malformed TUM list row " +
                               std::to_string(line_number));
    }
    entries.push_back({timestamp, dataset_root / std::filesystem::path(relative_path)});
  }

  std::stable_sort(entries.begin(), entries.end(),
                   [](const auto& lhs, const auto& rhs) {
                     return lhs.timestamp < rhs.timestamp;
                   });
  return entries;
}

AssociationResult associateNearest(const std::vector<TimestampedPath>& rgb,
                                   const std::vector<TimestampedPath>& depth,
                                   double max_skew_seconds) {
  if (!std::isfinite(max_skew_seconds) || max_skew_seconds < 0.0) {
    throw std::invalid_argument("max timestamp skew must be finite and non-negative");
  }

  auto sorted_rgb = rgb;
  auto sorted_depth = depth;
  std::stable_sort(sorted_rgb.begin(), sorted_rgb.end(),
                   [](const auto& lhs, const auto& rhs) {
                     return lhs.timestamp < rhs.timestamp;
                   });
  std::stable_sort(sorted_depth.begin(), sorted_depth.end(),
                   [](const auto& lhs, const auto& rhs) {
                     return lhs.timestamp < rhs.timestamp;
                   });

  std::vector<bool> used(sorted_depth.size(), false);
  AssociationResult result;
  for (const auto& rgb_entry : sorted_rgb) {
    std::size_t best_index = sorted_depth.size();
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < sorted_depth.size(); ++index) {
      if (used[index]) {
        continue;
      }
      const double distance =
          std::abs(rgb_entry.timestamp - sorted_depth[index].timestamp);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
      }
    }
    if (best_index != sorted_depth.size() &&
        best_distance <= max_skew_seconds + 1e-12) {
      used[best_index] = true;
      result.pairs.push_back({rgb_entry, sorted_depth[best_index]});
    } else {
      ++result.unmatched_rgb;
    }
  }
  result.unmatched_depth = static_cast<std::size_t>(
      std::count(used.begin(), used.end(), false));
  return result;
}

RgbdFrame loadRgbdFrame(const FramePair& pair, const CameraModel& camera) {
  camera.validate();
  cv::Mat color = cv::imread(pair.rgb.path.string(), cv::IMREAD_COLOR);
  cv::Mat depth = cv::imread(pair.depth.path.string(), cv::IMREAD_UNCHANGED);
  if (color.empty()) {
    throw std::runtime_error("cannot load RGB image: " + pair.rgb.path.string());
  }
  if (depth.empty()) {
    throw std::runtime_error("cannot load depth image: " + pair.depth.path.string());
  }
  if (color.cols != camera.width || color.rows != camera.height ||
      depth.cols != camera.width || depth.rows != camera.height ||
      color.size() != depth.size()) {
    throw std::runtime_error("RGB-D image resolution does not match calibration");
  }
  if (color.type() != CV_8UC3 || depth.type() != CV_16UC1) {
    throw std::runtime_error("expected 8-bit BGR and 16-bit single-channel depth");
  }
  return {pair.rgb.timestamp, pair.depth.timestamp, std::move(color),
          std::move(depth), pair.rgb.path, pair.depth.path};
}

}  // namespace sceneslam
