#include "sceneslam/odometry.hpp"
#include "test_support.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <vector>

using namespace sceneslam;

namespace {

double radians(double degrees) { return degrees * std::acos(-1.0) / 180.0; }

}  // namespace

int main() {
  int failures = 0;
  const CameraModel camera{525.0, 525.0, 319.5, 239.5, 640, 480,
                           5000.0, 0.1, 8.0};

  failures += runTest("PnP recovers metric pose with outliers", [&] {
    const double angle = radians(5.0);
    const cv::Matx33d rotation(std::cos(angle), 0.0, std::sin(angle),
                              0.0, 1.0, 0.0,
                              -std::sin(angle), 0.0, std::cos(angle));
    const Pose expected = poseFromRotationTranslation(rotation, {0.10, -0.02, 0.03});
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;
    for (int row = -3; row <= 3; ++row) {
      for (int col = -4; col <= 4; ++col) {
        const cv::Point3d point{col * 0.12, row * 0.10,
                                2.5 + 0.08 * ((row + col + 8) % 4)};
        object_points.push_back(point);
        cv::Point2d pixel = camera.project(transformPoint(expected, point));
        if (object_points.size() % 7 == 0) {
          pixel.x += 90.0;
          pixel.y -= 70.0;
        }
        image_points.push_back(pixel);
      }
    }

    cv::setRNGSeed(7);
    OdometryConfig config;
    config.min_correspondences = 8;
    config.min_inliers = 35;
    config.min_image_extent_px = 40.0;
    config.ransac_reprojection_error_px = 2.0;
    config.max_mean_reprojection_error_px = 0.5;
    const TrackingResult result =
        estimatePosePnP(object_points, image_points, camera, config);
    CHECK_TRUE(result.status == TrackingStatus::Tracking);
    CHECK_TRUE(result.pnp_inliers >= 50);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        checkNear(result.T_current_from_reference(row, col), expected(row, col),
                  0.01, "pose element");
      }
    }
  });

  failures += runTest("PnP rejects insufficient and poorly distributed points", [&] {
    OdometryConfig config;
    config.min_correspondences = 6;
    config.min_inliers = 6;
    const auto insufficient = estimatePosePnP(
        {{0, 0, 2}, {0.1, 0, 2}, {0, 0.1, 2}},
        {{320, 240}, {345, 240}, {320, 265}}, camera, config);
    CHECK_TRUE(insufficient.status == TrackingStatus::Lost);

    std::vector<cv::Point3d> objects;
    std::vector<cv::Point2d> images;
    for (int index = 0; index < 8; ++index) {
      objects.emplace_back(index * 0.01, 0.0, 2.0 + index * 0.01);
      images.emplace_back(320.0 + index * 0.1, 240.0 + index * 0.1);
    }
    config.min_image_extent_px = 20.0;
    const auto clustered = estimatePosePnP(objects, images, camera, config);
    CHECK_TRUE(clustered.status == TrackingStatus::Lost);
  });

  failures += runTest("PnP rejects correspondence sets dominated by outliers", [&] {
    std::vector<cv::Point3d> objects;
    std::vector<cv::Point2d> images;
    for (int index = 0; index < 20; ++index) {
      objects.emplace_back((index % 5) * 0.15, (index / 5) * 0.15,
                           2.0 + 0.03 * index);
      images.emplace_back(50.0 + 23.0 * index, 430.0 - 17.0 * index);
    }
    OdometryConfig config;
    config.min_correspondences = 8;
    config.min_inliers = 15;
    config.min_image_extent_px = 20.0;
    config.ransac_reprojection_error_px = 1.0;
    const auto result = estimatePosePnP(objects, images, camera, config);
    CHECK_TRUE(result.status == TrackingStatus::Lost);
  });

  failures += runTest("RGB-D tracker recovers identity on repeated textured frame", [&] {
    cv::Mat color(480, 640, CV_8UC3, cv::Scalar::all(0));
    for (int y = 50; y < 440; y += 45) {
      for (int x = 50; x < 600; x += 50) {
        const int shade = 80 + ((x + y) % 170);
        cv::rectangle(color, {x - 9, y - 7}, {x + 9, y + 7},
                      cv::Scalar(shade, 255 - shade, 160), 2);
        cv::line(color, {x - 7, y + 8}, {x + 8, y - 6},
                 cv::Scalar(255, 255, 255), 1);
      }
    }
    const cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(10000));
    const RgbdFrame frame{1.0, 1.0, color, depth, "rgb.png", "depth.png"};
    OdometryConfig config;
    config.min_correspondences = 20;
    config.min_inliers = 15;
    config.min_image_extent_px = 100.0;
    RgbdOdometry odometry(camera, config);
    const TrackingResult result = odometry.track(frame, frame);
    CHECK_TRUE(result.status == TrackingStatus::Tracking);
    CHECK_TRUE(result.matches >= 20);
    CHECK_TRUE(result.valid_depth_features >= 20);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        checkNear(result.T_current_from_reference(row, col),
                  row == col ? 1.0 : 0.0, 0.01, "identity pose");
      }
    }
  });

  return failures == 0 ? 0 : 1;
}
