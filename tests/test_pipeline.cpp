#include "sceneslam/pipeline.hpp"
#include "test_support.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

using namespace sceneslam;
namespace fs = std::filesystem;

namespace {

class TempDirectory {
 public:
  TempDirectory()
      : path_(fs::temp_directory_path() /
              ("sceneslam-pipeline-test-" +
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
  return runTest("pipeline records tracking loss and writes partial artifacts", [] {
    TempDirectory temporary;
    const fs::path dataset = temporary.path() / "dataset";
    const fs::path output = temporary.path() / "out";
    fs::create_directories(dataset / "rgb");
    fs::create_directories(dataset / "depth");

    cv::Mat textured(240, 320, CV_8UC3, cv::Scalar::all(0));
    for (int y = 40; y < 220; y += 30) {
      for (int x = 40; x < 300; x += 35) {
        cv::rectangle(textured, {x - 8, y - 6}, {x + 8, y + 6},
                      cv::Scalar((x + y) % 255, 200, 255), 2);
        cv::line(textured, {x - 6, y + 7}, {x + 7, y - 5},
                 cv::Scalar::all(255), 1);
      }
    }
    const cv::Mat blank(240, 320, CV_8UC3, cv::Scalar::all(0));
    const cv::Mat depth(240, 320, CV_16UC1, cv::Scalar(2000));
    const cv::Mat no_depth(240, 320, CV_16UC1, cv::Scalar(0));
    CHECK_TRUE(cv::imwrite((dataset / "rgb/1.png").string(), textured));
    CHECK_TRUE(cv::imwrite((dataset / "rgb/2.png").string(), textured));
    CHECK_TRUE(cv::imwrite((dataset / "rgb/3.png").string(), blank));
    CHECK_TRUE(cv::imwrite((dataset / "depth/1.png").string(), depth));
    CHECK_TRUE(cv::imwrite((dataset / "depth/2.png").string(), depth));
    CHECK_TRUE(cv::imwrite((dataset / "depth/3.png").string(), no_depth));
    {
      std::ofstream rgb(dataset / "rgb.txt");
      std::ofstream dep(dataset / "depth.txt");
      for (int index = 1; index <= 3; ++index) {
        rgb << index << ".000 rgb/" << index << ".png\n";
        dep << index << ".005 depth/" << index << ".png\n";
      }
    }
    const fs::path config_path = temporary.path() / "camera.yaml";
    {
      std::ofstream config(config_path);
      config << "%YAML:1.0\nfx: 250.0\nfy: 250.0\ncx: 159.5\ncy: 119.5\n"
                "width: 320\nheight: 240\ndepth_units_per_metre: 1000.0\n"
                "min_depth_m: 0.1\nmax_depth_m: 8.0\n";
    }

    RunOptions options;
    options.dataset = dataset;
    options.camera_config = config_path;
    options.output = output;
    options.max_frames = 3;
    options.max_timestamp_skew_seconds = 0.01;
    options.map_config = {0.05, 50};
    options.map_pixel_stride = 8;
    options.odometry_config.min_correspondences = 15;
    options.odometry_config.min_inliers = 12;
    options.odometry_config.min_image_extent_px = 60.0;
    const RunSummary summary = runDataset(options);
    CHECK_TRUE(summary.paired_frames == 3);
    CHECK_TRUE(summary.tracked_frames == 2);
    CHECK_TRUE(summary.lost_frames == 1);
    CHECK_TRUE(summary.map_points <= 50);
    CHECK_TRUE(fs::exists(output / "trajectory_tum.txt"));
    CHECK_TRUE(fs::exists(output / "frames.jsonl"));
    CHECK_TRUE(fs::exists(output / "map.ply"));
    CHECK_TRUE(fs::exists(output / "summary.json"));
    const std::string trajectory = readText(output / "trajectory_tum.txt");
    CHECK_TRUE(std::count(trajectory.begin(), trajectory.end(), '\n') == 2);
    const std::string frames = readText(output / "frames.jsonl");
    CHECK_TRUE(frames.find("\"status\":\"LOST\"") != std::string::npos);
  });
}
