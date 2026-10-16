#pragma once

#include "sceneslam/camera.hpp"
#include "sceneslam/geometry.hpp"
#include "sceneslam/rgbd.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <tuple>
#include <vector>

namespace sceneslam {

struct ColoredPoint {
  double x{};
  double y{};
  double z{};
  std::uint8_t red{};
  std::uint8_t green{};
  std::uint8_t blue{};
};

struct VoxelMapConfig {
  double voxel_size_m{0.03};
  std::size_t max_points{500000};
};

class VoxelMap {
 public:
  explicit VoxelMap(VoxelMapConfig config = {});
  void integrate(const RgbdFrame& frame, const CameraModel& camera,
                 const Pose& T_world_from_camera, int pixel_stride = 4);
  [[nodiscard]] std::size_t size() const;
  [[nodiscard]] std::vector<ColoredPoint> points() const;
  void writePly(const std::filesystem::path& path) const;

 private:
  using VoxelKey = std::tuple<long long, long long, long long>;
  VoxelMapConfig config_;
  std::map<VoxelKey, ColoredPoint> voxels_;
};

}  // namespace sceneslam

