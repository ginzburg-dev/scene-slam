# SceneSLAM developer tasks. Run `make` for the list.
#
# Override any variable on the command line, for example:
#   make build OPENCV_DIR=/opt/opencv/lib/cmake/opencv4
#   make run DATASET=/data/rgbd_dataset_freiburg1_xyz OUT=out/fr1_xyz

BUILD_DIR ?= build
CONFIG ?= Release
OPENCV_DIR ?=

DATASET ?= data/rgbd_dataset_freiburg1_room
CAMERA ?= configs/tum_freiburg1.yaml
OUT ?= out/fr1_room
GROUNDTRUTH ?= $(DATASET)/groundtruth.txt
VOXEL_SIZE ?= 0.02
MAX_POINTS ?= 4000000
MAX_FRAMES ?=

# Camera for `make screenshot`, in the world frame (= the first camera frame).
EYE ?= 5.114,-7.142,1.236
LOOK_AT ?= 0.33,0.0,0.9
UP ?= -0.0327,-0.7529,-0.6573
FRUSTUM_STEP ?= 25
WINDOW_SIZE ?= 1600x1000

ifeq ($(OS),Windows_NT)
  SCENE_SLAM := $(BUILD_DIR)/$(CONFIG)/scene_slam.exe
else
  SCENE_SLAM := $(BUILD_DIR)/scene_slam
endif

CMAKE_FLAGS := -DCMAKE_BUILD_TYPE=$(CONFIG)
ifneq ($(OPENCV_DIR),)
  CMAKE_FLAGS += -DOpenCV_DIR=$(OPENCV_DIR)
endif

RUN_FLAGS := --dataset $(DATASET) --camera $(CAMERA) --out $(OUT) \
             --voxel-size $(VOXEL_SIZE) --max-points $(MAX_POINTS)
ifneq ($(MAX_FRAMES),)
  RUN_FLAGS += --max-frames $(MAX_FRAMES)
endif

.DEFAULT_GOAL := help
.PHONY: help install configure build test selftest run view screenshot eval clean distclean

help: ## List the available targets
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  %-12s %s\n", $$1, $$2}' $(MAKEFILE_LIST)

install: ## Create the Python environment for the viewer and evaluation (uv sync)
	uv sync

configure: ## Configure the CMake build (pass OPENCV_DIR if OpenCV is not on the default path)
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)

build: configure ## Build scene_slam and the test binaries
	cmake --build $(BUILD_DIR) --config $(CONFIG)

test: build ## Run the full ctest suite
	ctest --test-dir $(BUILD_DIR) -C $(CONFIG) --output-on-failure

selftest: build ## Run the built-in geometry self-test
	$(SCENE_SLAM) selftest

run: build ## Track the DATASET sequence into the OUT directory
	$(SCENE_SLAM) run $(RUN_FLAGS)

view: ## Open the OUT directory in the Rerun viewer
	uv run python scripts/view_map.py $(OUT)

screenshot: ## Render OUT to overview.png with the published camera
	uv run python scripts/view_map.py $(OUT) --save $(OUT)/recording.rrd \
		--frustum-step $(FRUSTUM_STEP) --hide-panels --hide-grid \
		--eye=$(EYE) --look-at=$(LOOK_AT) --up=$(UP)
	uv run rerun $(OUT)/recording.rrd --hide-welcome-screen \
		--screenshot-to $(OUT)/overview.png --window-size $(WINDOW_SIZE)

eval: ## Compare the OUT trajectory with GROUNDTRUTH using evo
	uv run --with evo python scripts/evaluate.py \
		--groundtruth $(GROUNDTRUTH) \
		--trajectory $(OUT)/trajectory_tum.txt --execute

clean: ## Remove build outputs
	cmake -E rm -rf $(BUILD_DIR)

distclean: clean ## Remove build outputs, run results, and the Python environment
	cmake -E rm -rf out .venv
