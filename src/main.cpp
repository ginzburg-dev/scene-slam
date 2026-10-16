#include "sceneslam/pipeline.hpp"

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef SCENESLAM_VERSION
#define SCENESLAM_VERSION "0.0.0-dev"
#endif

namespace {

void printUsage() {
  std::cout << "SceneSLAM " SCENESLAM_VERSION "\n"
               "Usage:\n"
               "  scene_slam --version\n"
               "  scene_slam selftest\n"
               "  scene_slam run --dataset PATH --camera FILE --out PATH "
               "[--max-frames N] [--frame-step N] [--max-skew SECONDS] "
               "[--voxel-size METRES] [--max-points N]\n";
}

std::string requireValue(int& index, int argc, char** argv) {
  if (++index >= argc) {
    throw std::invalid_argument("missing value for command-line option");
  }
  return argv[index];
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2) {
      printUsage();
      return 2;
    }
    const std::string command = argv[1];
    if (command == "--version" || command == "-v") {
      std::cout << "SceneSLAM " SCENESLAM_VERSION "\n";
      return 0;
    }
    if (command == "selftest") {
      if (!sceneslam::runSelfTest()) {
        std::cerr << "SceneSLAM geometry self-test failed\n";
        return 1;
      }
      std::cout << "SceneSLAM geometric self-tests passed\n";
      return 0;
    }
    if (command != "run") {
      printUsage();
      return 2;
    }

    sceneslam::RunOptions options;
    for (int index = 2; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--dataset") {
        options.dataset = requireValue(index, argc, argv);
      } else if (argument == "--camera") {
        options.camera_config = requireValue(index, argc, argv);
      } else if (argument == "--out") {
        options.output = requireValue(index, argc, argv);
      } else if (argument == "--max-frames") {
        options.max_frames = std::stoull(requireValue(index, argc, argv));
      } else if (argument == "--frame-step") {
        options.frame_step = std::stoull(requireValue(index, argc, argv));
      } else if (argument == "--max-skew") {
        options.max_timestamp_skew_seconds =
            std::stod(requireValue(index, argc, argv));
      } else if (argument == "--voxel-size") {
        options.map_config.voxel_size_m =
            std::stod(requireValue(index, argc, argv));
      } else if (argument == "--max-points") {
        options.map_config.max_points =
            std::stoull(requireValue(index, argc, argv));
      } else {
        throw std::invalid_argument("unknown option: " + argument);
      }
    }

    const sceneslam::RunSummary summary = sceneslam::runDataset(options);
    std::cout << "RGB=" << summary.rgb_frames << " depth=" << summary.depth_frames
              << " paired=" << summary.paired_frames
              << " tracked=" << summary.tracked_frames
              << " lost=" << summary.lost_frames
              << " skipped_rgb=" << summary.skipped_rgb_frames
              << " unmatched_depth=" << summary.unmatched_depth_frames
              << " map_points=" << summary.map_points
              << " success=" << summary.tracking_success_percent << "%"
              << " mean_fps=" << summary.mean_fps << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}

