#include "sceneslam/rgbd.hpp"
#include "test_support.hpp"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace sceneslam;
namespace fs = std::filesystem;

namespace {

class TempDirectory {
 public:
  TempDirectory() {
    path_ = fs::temp_directory_path() /
            ("sceneslam-test-" +
             std::to_string(std::chrono::steady_clock::now()
                                .time_since_epoch()
                                .count()));
    fs::create_directories(path_);
  }
  ~TempDirectory() { std::error_code error; fs::remove_all(path_, error); }
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

}  // namespace

int main() {
  int failures = 0;

  failures += runTest("TUM list parser handles comments and rejects malformed rows", [] {
    TempDirectory directory;
    const fs::path list = directory.path() / "rgb.txt";
    {
      std::ofstream stream(list);
      stream << "# comment\n\n1.250 rgb/one.png\n2.500 rgb/two.png\n";
    }
    const auto entries = readTumList(list, directory.path());
    CHECK_TRUE(entries.size() == 2);
    checkNear(entries[0].timestamp, 1.25, 1e-12, "timestamp");
    CHECK_TRUE(entries[0].path == directory.path() / "rgb/one.png");

    {
      std::ofstream stream(list);
      stream << "not-a-time rgb/one.png\n";
    }
    bool rejected = false;
    try {
      (void)readTumList(list, directory.path());
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    CHECK_TRUE(rejected);
  });

  failures += runTest("association is nearest unused deterministic and skew bounded", [] {
    const std::vector<TimestampedPath> rgb{{1.00, "r0"}, {1.04, "r1"},
                                           {2.00, "r2"}};
    const std::vector<TimestampedPath> depth{{0.99, "d0"}, {1.01, "d1"},
                                             {1.05, "d2"}, {2.03, "d3"}};
    const AssociationResult result = associateNearest(rgb, depth, 0.02);
    CHECK_TRUE(result.pairs.size() == 2);
    CHECK_TRUE(result.pairs[0].rgb.path == "r0");
    CHECK_TRUE(result.pairs[0].depth.path == "d0");
    CHECK_TRUE(result.pairs[1].depth.path == "d2");
    CHECK_TRUE(result.unmatched_rgb == 1);
    CHECK_TRUE(result.unmatched_depth == 2);

    const std::vector<TimestampedPath> duplicate_depth{{3.0, "first"},
                                                       {3.0, "second"}};
    const auto duplicate = associateNearest({{3.0, "rgb"}}, duplicate_depth, 0.0);
    CHECK_TRUE(duplicate.pairs.size() == 1);
    CHECK_TRUE(duplicate.pairs[0].depth.path == "first");
  });

  failures += runTest("RGB-D loader rejects mismatched resolution and missing files", [] {
    TempDirectory directory;
    const fs::path rgb_path = directory.path() / "rgb.png";
    const fs::path depth_path = directory.path() / "depth.png";
    CHECK_TRUE(cv::imwrite(rgb_path.string(), cv::Mat(2, 2, CV_8UC3, cv::Scalar(1, 2, 3))));
    CHECK_TRUE(cv::imwrite(depth_path.string(), cv::Mat(1, 2, CV_16UC1, cv::Scalar(5000))));
    const CameraModel camera{500.0, 500.0, 1.0, 1.0, 2, 2, 5000.0, 0.1, 8.0};
    bool mismatch_rejected = false;
    try {
      (void)loadRgbdFrame({{1.0, rgb_path}, {1.0, depth_path}}, camera);
    } catch (const std::runtime_error&) {
      mismatch_rejected = true;
    }
    CHECK_TRUE(mismatch_rejected);

    bool missing_rejected = false;
    try {
      (void)loadRgbdFrame({{1.0, directory.path() / "missing.png"},
                           {1.0, depth_path}}, camera);
    } catch (const std::runtime_error&) {
      missing_rejected = true;
    }
    CHECK_TRUE(missing_rejected);
  });

  return failures == 0 ? 0 : 1;
}
