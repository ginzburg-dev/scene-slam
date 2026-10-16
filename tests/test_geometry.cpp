#include "sceneslam/camera.hpp"
#include "sceneslam/geometry.hpp"
#include "test_support.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

using namespace sceneslam;

int main() {
  int failures = 0;

  failures += runTest("camera validation and metric backprojection", [] {
    CameraModel camera{500.0, 500.0, 320.0, 240.0, 640, 480, 5000.0, 0.1,
                       8.0};
    camera.validate();
    const cv::Point3d point = camera.backProject({420.0, 190.0}, 10000);
    checkNear(point.x, 0.4, 1e-12, "x");
    checkNear(point.y, -0.2, 1e-12, "y");
    checkNear(point.z, 2.0, 1e-12, "z");
    const cv::Point2d pixel = camera.project(point);
    checkNear(pixel.x, 420.0, 1e-12, "u");
    checkNear(pixel.y, 190.0, 1e-12, "v");

    bool rejected = false;
    try {
      CameraModel{0.0, 500.0, 320.0, 240.0, 640, 480, 5000.0, 0.1, 8.0}
          .validate();
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    CHECK_TRUE(rejected);
    CHECK_TRUE(!camera.backProjectIfValid({0.0, 0.0}, 0).has_value());
  });

  failures += runTest("SE3 inverse composition and point transform", [] {
    const Pose pose = poseFromRotationTranslation(
        cv::Matx33d(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        {1.0, 2.0, 3.0});
    const Pose identity = composePose(pose, invertPose(pose));
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        checkNear(identity(row, col), row == col ? 1.0 : 0.0, 1e-12,
                  "identity");
      }
    }
    const cv::Point3d transformed = transformPoint(pose, {1.0, 0.0, 0.0});
    checkNear(transformed.x, 1.0, 1e-12, "transformed x");
    checkNear(transformed.y, 3.0, 1e-12, "transformed y");
    checkNear(transformed.z, 3.0, 1e-12, "transformed z");
  });

  failures += runTest("quaternion uses xyzw order", [] {
    const Pose pose = poseFromRotationTranslation(
        cv::Matx33d(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        {0.0, 0.0, 0.0});
    const std::array<double, 4> quaternion = rotationToQuaternionXyzw(pose);
    checkNear(quaternion[0], 0.0, 1e-12, "qx");
    checkNear(quaternion[1], 0.0, 1e-12, "qy");
    checkNear(quaternion[2], std::sqrt(0.5), 1e-12, "qz");
    checkNear(quaternion[3], std::sqrt(0.5), 1e-12, "qw");
  });

  return failures == 0 ? 0 : 1;
}
