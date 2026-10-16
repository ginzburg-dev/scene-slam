#include "sceneslam/pipeline.hpp"

#include "sceneslam/rgbd.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace sceneslam {
namespace {

template <class T>
T readRequired(const cv::FileStorage& storage, const char* name) {
  const cv::FileNode node = storage[name];
  if (node.empty()) {
    throw std::runtime_error(std::string("missing camera setting: ") + name);
  }
  T value{};
  node >> value;
  return value;
}

void writeDebugFrame(const std::filesystem::path& directory, std::size_t index,
                     const RgbdFrame& frame, const FrameRecord& record) {
  cv::Mat image = frame.color.clone();
  const std::string status =
      record.status == TrackingStatus::Tracking ? "TRACKING" : "LOST";
  cv::putText(image, status + " matches=" + std::to_string(record.matches) +
                         " inliers=" + std::to_string(record.pnp_inliers),
              {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
              record.status == TrackingStatus::Tracking ? cv::Scalar(0, 220, 0)
                                                         : cv::Scalar(0, 0, 255),
              2, cv::LINE_AA);
  std::filesystem::create_directories(directory);
  const auto path = directory / ("frame_" + std::to_string(index) + ".png");
  if (!cv::imwrite(path.string(), image)) {
    throw std::runtime_error("cannot write debug image: " + path.string());
  }
}

}  // namespace

CameraModel loadCameraConfig(const std::filesystem::path& config_path) {
  cv::FileStorage storage(config_path.string(), cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error("cannot open camera config: " + config_path.string());
  }
  CameraModel camera;
  camera.fx = readRequired<double>(storage, "fx");
  camera.fy = readRequired<double>(storage, "fy");
  camera.cx = readRequired<double>(storage, "cx");
  camera.cy = readRequired<double>(storage, "cy");
  camera.width = readRequired<int>(storage, "width");
  camera.height = readRequired<int>(storage, "height");
  camera.depth_units_per_metre =
      readRequired<double>(storage, "depth_units_per_metre");
  if (!storage["min_depth_m"].empty()) {
    storage["min_depth_m"] >> camera.min_depth_m;
  }
  if (!storage["max_depth_m"].empty()) {
    storage["max_depth_m"] >> camera.max_depth_m;
  }
  camera.validate();
  return camera;
}

RunSummary runDataset(const RunOptions& options) {
  if (options.dataset.empty() || options.camera_config.empty() ||
      options.output.empty() || options.frame_step == 0 ||
      options.map_pixel_stride <= 0 || options.debug_frame_stride <= 0) {
    throw std::invalid_argument("invalid run options");
  }
  const CameraModel camera = loadCameraConfig(options.camera_config);
  const auto rgb = readTumList(options.dataset / "rgb.txt", options.dataset);
  const auto depth = readTumList(options.dataset / "depth.txt", options.dataset);
  const AssociationResult association =
      associateNearest(rgb, depth, options.max_timestamp_skew_seconds);
  if (association.pairs.empty()) {
    throw std::runtime_error("no RGB-D frame pairs within timestamp skew");
  }

  std::filesystem::create_directories(options.output);
  RgbdOdometry odometry(camera, options.odometry_config);
  VoxelMap map(options.map_config);
  std::vector<FrameRecord> records;
  std::optional<RgbdFrame> reference_frame;
  Pose T_world_from_reference = identityPose();

  std::size_t selected = 0;
  for (std::size_t pair_index = 0; pair_index < association.pairs.size();
       pair_index += options.frame_step) {
    if (options.max_frames != 0 && selected >= options.max_frames) {
      break;
    }
    ++selected;
    const auto started = std::chrono::steady_clock::now();
    FrameRecord record;
    record.timestamp = association.pairs[pair_index].rgb.timestamp;
    try {
      RgbdFrame frame = loadRgbdFrame(association.pairs[pair_index], camera);
      if (!reference_frame) {
        record.status = TrackingStatus::Tracking;
        record.T_world_from_camera = identityPose();
        map.integrate(frame, camera, *record.T_world_from_camera,
                      options.map_pixel_stride);
        reference_frame = frame;
        T_world_from_reference = *record.T_world_from_camera;
      } else {
        const TrackingResult tracking = odometry.track(*reference_frame, frame);
        record.status = tracking.status;
        record.valid_depth_features = tracking.valid_depth_features;
        record.matches = tracking.matches;
        record.pnp_inliers = tracking.pnp_inliers;
        record.reprojection_error_px = tracking.reprojection_error_px;
        record.relative_translation_m = tracking.relative_translation_m;
        record.relative_rotation_deg = tracking.relative_rotation_deg;
        record.reason = tracking.reason;
        if (tracking.status == TrackingStatus::Tracking) {
          record.T_world_from_camera = composePose(
              T_world_from_reference,
              invertPose(tracking.T_current_from_reference));
          map.integrate(frame, camera, *record.T_world_from_camera,
                        options.map_pixel_stride);
          reference_frame = frame;
          T_world_from_reference = *record.T_world_from_camera;
        }
      }
      record.tracking_time_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
      if ((selected - 1) % static_cast<std::size_t>(options.debug_frame_stride) ==
          0) {
        writeDebugFrame(options.output / "debug", selected - 1, frame, record);
      }
    } catch (const std::exception& error) {
      record.status = TrackingStatus::Lost;
      record.reason = error.what();
      record.tracking_time_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
    }
    records.push_back(std::move(record));
  }

  RunSummary summary;
  summary.rgb_frames = rgb.size();
  summary.depth_frames = depth.size();
  summary.paired_frames = selected;
  for (const auto& record : records) {
    if (record.status == TrackingStatus::Tracking && record.T_world_from_camera) {
      ++summary.tracked_frames;
    } else {
      ++summary.lost_frames;
    }
  }
  summary.map_points = map.size();
  writeTumTrajectory(options.output / "trajectory_tum.txt", records);
  writeFramesJsonl(options.output / "frames.jsonl", records);
  map.writePly(options.output / "map.ply");
  writeSummaryJson(options.output / "summary.json", summary);
  return summary;
}

bool runSelfTest() {
  const CameraModel camera{500.0, 500.0, 320.0, 240.0, 640, 480,
                           5000.0, 0.1, 8.0};
  camera.validate();
  const cv::Point3d point = camera.backProject({420.0, 190.0}, 10000);
  const cv::Point2d pixel = camera.project(point);
  const Pose pose = poseFromRotationTranslation(cv::Matx33d::eye(), {1.0, 2.0, 3.0});
  const Pose identity = composePose(pose, invertPose(pose));
  return std::abs(pixel.x - 420.0) < 1e-9 &&
         std::abs(pixel.y - 190.0) < 1e-9 &&
         cv::norm(cv::Mat(identity - Pose::eye())) < 1e-9;
}

}  // namespace sceneslam

