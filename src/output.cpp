#include "sceneslam/output.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace sceneslam {
namespace {

std::ofstream openOutput(const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot write output: " + path.string());
  }
  return stream;
}

std::string escapeJson(const std::string& value) {
  std::string escaped;
  for (const char character : value) {
    switch (character) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character; break;
    }
  }
  return escaped;
}

}  // namespace

void writeTumTrajectory(const std::filesystem::path& path,
                        const std::vector<FrameRecord>& records) {
  auto stream = openOutput(path);
  stream << std::fixed << std::setprecision(9);
  for (const auto& record : records) {
    if (record.status != TrackingStatus::Tracking || !record.T_world_from_camera) {
      continue;
    }
    const Pose& pose = *record.T_world_from_camera;
    const auto quaternion = rotationToQuaternionXyzw(pose);
    stream << record.timestamp << ' ' << pose(0, 3) << ' ' << pose(1, 3) << ' '
           << pose(2, 3) << ' ' << quaternion[0] << ' ' << quaternion[1] << ' '
           << quaternion[2] << ' ' << quaternion[3] << '\n';
  }
}

void writeFramesJsonl(const std::filesystem::path& path,
                      const std::vector<FrameRecord>& records) {
  auto stream = openOutput(path);
  stream << std::fixed << std::setprecision(6);
  for (const auto& record : records) {
    stream << "{\"timestamp\":" << record.timestamp << ",\"status\":\""
           << (record.status == TrackingStatus::Tracking ? "TRACKING" : "LOST")
           << "\",\"valid_depth_features\":" << record.valid_depth_features
           << ",\"matches\":" << record.matches << ",\"pnp_inliers\":"
           << record.pnp_inliers << ",\"reprojection_error_px\":"
           << record.reprojection_error_px << ",\"relative_translation_m\":"
           << record.relative_translation_m << ",\"relative_rotation_deg\":"
           << record.relative_rotation_deg << ",\"tracking_time_ms\":"
           << record.tracking_time_ms << ",\"reason\":\""
           << escapeJson(record.reason) << "\"";
    if (record.T_world_from_camera) {
      const Pose& pose = *record.T_world_from_camera;
      const auto quaternion = rotationToQuaternionXyzw(pose);
      stream << ",\"pose\":{"
             << "\"tx\":" << pose(0, 3) << ",\"ty\":" << pose(1, 3)
             << ",\"tz\":" << pose(2, 3) << ",\"qx\":" << quaternion[0]
             << ",\"qy\":" << quaternion[1] << ",\"qz\":" << quaternion[2]
             << ",\"qw\":" << quaternion[3] << '}';
    }
    stream << "}\n";
  }
}

void writeSummaryJson(const std::filesystem::path& path,
                      const RunSummary& summary) {
  auto stream = openOutput(path);
  stream << "{\n"
         << "  \"rgb_frames\": " << summary.rgb_frames << ",\n"
         << "  \"depth_frames\": " << summary.depth_frames << ",\n"
         << "  \"paired_frames\": " << summary.paired_frames << ",\n"
         << "  \"tracked_frames\": " << summary.tracked_frames << ",\n"
         << "  \"lost_frames\": " << summary.lost_frames << ",\n"
         << "  \"map_points\": " << summary.map_points << ",\n"
         << "  \"skipped_rgb_frames\": " << summary.skipped_rgb_frames << ",\n"
         << "  \"unmatched_depth_frames\": "
         << summary.unmatched_depth_frames << "\n}\n";
}

}  // namespace sceneslam
