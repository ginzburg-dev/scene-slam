#include "sceneslam/map.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace sceneslam {

VoxelMap::VoxelMap(VoxelMapConfig config) : config_(config) {
  if (!std::isfinite(config_.voxel_size_m) || config_.voxel_size_m <= 0.0 ||
      config_.max_points == 0) {
    throw std::invalid_argument("invalid voxel map configuration");
  }
}

void VoxelMap::integrate(const RgbdFrame& frame, const CameraModel& camera,
                         const Pose& T_world_from_camera, int pixel_stride) {
  camera.validate();
  if (pixel_stride <= 0 || frame.color.type() != CV_8UC3 ||
      frame.depth.type() != CV_16UC1 || frame.color.size() != frame.depth.size()) {
    throw std::invalid_argument("invalid RGB-D frame for map integration");
  }
  for (int row = 0; row < frame.depth.rows; row += pixel_stride) {
    for (int col = 0; col < frame.depth.cols; col += pixel_stride) {
      const auto camera_point = camera.backProjectIfValid(
          {static_cast<double>(col), static_cast<double>(row)},
          frame.depth.at<std::uint16_t>(row, col));
      if (!camera_point) {
        continue;
      }
      const cv::Point3d world_point = transformPoint(T_world_from_camera, *camera_point);
      const VoxelKey key{
          static_cast<long long>(std::floor(world_point.x / config_.voxel_size_m)),
          static_cast<long long>(std::floor(world_point.y / config_.voxel_size_m)),
          static_cast<long long>(std::floor(world_point.z / config_.voxel_size_m))};
      if (voxels_.contains(key) || voxels_.size() >= config_.max_points) {
        continue;
      }
      const cv::Vec3b color = frame.color.at<cv::Vec3b>(row, col);
      voxels_.emplace(key, ColoredPoint{world_point.x, world_point.y, world_point.z,
                                       color[2], color[1], color[0]});
    }
  }
}

std::size_t VoxelMap::size() const { return voxels_.size(); }

std::vector<ColoredPoint> VoxelMap::points() const {
  std::vector<ColoredPoint> result;
  result.reserve(voxels_.size());
  for (const auto& [key, point] : voxels_) {
    (void)key;
    result.push_back(point);
  }
  return result;
}

void VoxelMap::writePly(const std::filesystem::path& path) const {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot write PLY: " + path.string());
  }
  stream << "ply\nformat ascii 1.0\nelement vertex " << voxels_.size()
         << "\nproperty float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
  stream << std::setprecision(9);
  for (const auto& [key, point] : voxels_) {
    (void)key;
    stream << point.x << ' ' << point.y << ' ' << point.z << ' '
           << static_cast<int>(point.red) << ' ' << static_cast<int>(point.green)
           << ' ' << static_cast<int>(point.blue) << '\n';
  }
}

}  // namespace sceneslam

