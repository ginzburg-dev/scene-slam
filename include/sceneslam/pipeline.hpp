#pragma once

#include "sceneslam/map.hpp"
#include "sceneslam/odometry.hpp"
#include "sceneslam/output.hpp"

#include <cstddef>
#include <filesystem>

namespace sceneslam {

struct RunOptions {
  std::filesystem::path dataset;
  std::filesystem::path camera_config;
  std::filesystem::path output;
  std::size_t max_frames{};
  std::size_t frame_step{1};
  double max_timestamp_skew_seconds{0.02};
  VoxelMapConfig map_config{};
  int map_pixel_stride{4};
  int debug_frame_stride{50};
  OdometryConfig odometry_config{};
};

[[nodiscard]] CameraModel loadCameraConfig(
    const std::filesystem::path& config_path);
[[nodiscard]] RunSummary runDataset(const RunOptions& options);
[[nodiscard]] bool runSelfTest();

}  // namespace sceneslam

