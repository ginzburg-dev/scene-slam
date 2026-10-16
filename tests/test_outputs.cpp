#include "sceneslam/map.hpp"
#include "sceneslam/output.hpp"
#include "test_support.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace sceneslam;
namespace fs = std::filesystem;

namespace {

class TempDirectory {
 public:
  TempDirectory()
      : path_(fs::temp_directory_path() /
              ("sceneslam-output-test-" +
               std::to_string(std::chrono::steady_clock::now()
                                  .time_since_epoch()
                                  .count()))) {
    fs::create_directories(path_);
  }
  ~TempDirectory() { std::error_code error; fs::remove_all(path_, error); }
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

std::string readText(const fs::path& path) {
  std::ifstream stream(path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

}  // namespace

int main() {
  int failures = 0;

  failures += runTest("voxel map transforms points, omits invalid depth, and caps size", [] {
    const CameraModel camera{1.0, 1.0, 0.0, 0.0, 2, 2, 1000.0, 0.1, 8.0};
    cv::Mat color(2, 2, CV_8UC3, cv::Scalar(30, 20, 10));
    cv::Mat depth(2, 2, CV_16UC1, cv::Scalar(1000));
    depth.at<std::uint16_t>(0, 0) = 0;
    const RgbdFrame frame{1.0, 1.0, color, depth, "rgb", "depth"};
    VoxelMap map({0.01, 2});
    const Pose pose = poseFromRotationTranslation(cv::Matx33d::eye(), {1.0, 2.0, 3.0});
    map.integrate(frame, camera, pose, 1);
    CHECK_TRUE(map.size() == 2);
    map.integrate(frame, camera, pose, 1);
    CHECK_TRUE(map.size() == 2);
    const auto points = map.points();
    CHECK_TRUE(points[0].red == 10 && points[0].green == 20 && points[0].blue == 30);
    CHECK_TRUE(points[0].z >= 4.0 - 1e-12);
  });

  failures += runTest("PLY and run artifacts preserve valid poses and explicit loss", [] {
    TempDirectory directory;
    VoxelMap map({0.05, 10});
    const CameraModel camera{1.0, 1.0, 0.0, 0.0, 1, 1, 1000.0, 0.1, 8.0};
    const RgbdFrame frame{1.0, 1.0,
                          cv::Mat(1, 1, CV_8UC3, cv::Scalar(3, 2, 1)),
                          cv::Mat(1, 1, CV_16UC1, cv::Scalar(1000)), "rgb", "depth"};
    map.integrate(frame, camera, identityPose(), 1);
    map.writePly(directory.path() / "map.ply");
    const std::string ply = readText(directory.path() / "map.ply");
    CHECK_TRUE(ply.find("element vertex 1") != std::string::npos);
    CHECK_TRUE(ply.find("0 0 1 1 2 3") != std::string::npos);

    FrameRecord valid;
    valid.timestamp = 1.25;
    valid.status = TrackingStatus::Tracking;
    valid.T_world_from_camera =
        poseFromRotationTranslation(cv::Matx33d::eye(), {1.0, 2.0, 3.0});
    valid.matches = 20;
    valid.pnp_inliers = 18;
    FrameRecord lost;
    lost.timestamp = 1.5;
    lost.status = TrackingStatus::Lost;
    lost.reason = "bad \"depth\"\nrow";
    const std::vector<FrameRecord> records{valid, lost};
    writeTumTrajectory(directory.path() / "trajectory_tum.txt", records);
    writeFramesJsonl(directory.path() / "frames.jsonl", records);
    RunSummary run_summary{2, 2, 2, 1, 1, 1};
    run_summary.skipped_rgb_frames = 3;
    run_summary.unmatched_depth_frames = 4;
    run_summary.tracking_success_percent = 50.0;
    run_summary.median_tracking_time_ms = 12.5;
    run_summary.p95_tracking_time_ms = 20.0;
    run_summary.mean_fps = 80.0;
    run_summary.map_capacity_reached = true;
    writeSummaryJson(directory.path() / "summary.json", run_summary);

    const std::string trajectory = readText(directory.path() / "trajectory_tum.txt");
    CHECK_TRUE(std::count(trajectory.begin(), trajectory.end(), '\n') == 1);
    CHECK_TRUE(trajectory.find("1.250000000 1.000000000 2.000000000 3.000000000 0.000000000 0.000000000 0.000000000 1.000000000") != std::string::npos);
    const std::string frames = readText(directory.path() / "frames.jsonl");
    CHECK_TRUE(std::count(frames.begin(), frames.end(), '\n') == 2);
    CHECK_TRUE(frames.find("\"status\":\"LOST\"") != std::string::npos);
    CHECK_TRUE(frames.find("bad \\\"depth\\\"\\nrow") != std::string::npos);
    const std::string summary = readText(directory.path() / "summary.json");
    CHECK_TRUE(summary.find("\"tracked_frames\": 1") != std::string::npos);
    CHECK_TRUE(summary.find("\"lost_frames\": 1") != std::string::npos);
    CHECK_TRUE(summary.find("\"skipped_rgb_frames\": 3") != std::string::npos);
    CHECK_TRUE(summary.find("\"unmatched_depth_frames\": 4") != std::string::npos);
    CHECK_TRUE(summary.find("\"tracking_success_percent\": 50") != std::string::npos);
    CHECK_TRUE(summary.find("\"median_tracking_time_ms\": 12.5") != std::string::npos);
    CHECK_TRUE(summary.find("\"p95_tracking_time_ms\": 20") != std::string::npos);
    CHECK_TRUE(summary.find("\"mean_fps\": 80") != std::string::npos);
    CHECK_TRUE(summary.find("\"map_capacity_reached\": true") != std::string::npos);
  });

  return failures == 0 ? 0 : 1;
}
