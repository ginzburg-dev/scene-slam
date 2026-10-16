#pragma once

#include "sceneslam/geometry.hpp"
#include "sceneslam/odometry.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace sceneslam {

struct FrameRecord {
  double timestamp{};
  TrackingStatus status{TrackingStatus::Lost};
  int valid_depth_features{};
  int matches{};
  int pnp_inliers{};
  double reprojection_error_px{};
  double relative_translation_m{};
  double relative_rotation_deg{};
  double tracking_time_ms{};
  std::optional<Pose> T_world_from_camera;
  std::string reason;
};

struct RunSummary {
  std::size_t rgb_frames{};
  std::size_t depth_frames{};
  std::size_t paired_frames{};
  std::size_t tracked_frames{};
  std::size_t lost_frames{};
  std::size_t map_points{};
  std::size_t skipped_rgb_frames{};
  std::size_t unmatched_depth_frames{};
};

void writeTumTrajectory(const std::filesystem::path& path,
                        const std::vector<FrameRecord>& records);
void writeFramesJsonl(const std::filesystem::path& path,
                      const std::vector<FrameRecord>& records);
void writeSummaryJson(const std::filesystem::path& path,
                      const RunSummary& summary);

}  // namespace sceneslam

