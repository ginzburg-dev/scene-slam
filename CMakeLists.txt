cmake_minimum_required(VERSION 3.20)
project(SceneSLAM VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d features2d imgcodecs imgproc)

add_library(sceneslam_core src/geometry.cpp src/rgbd.cpp src/odometry.cpp
  src/map.cpp src/output.cpp src/pipeline.cpp)

target_include_directories(sceneslam_core PUBLIC include)
target_link_libraries(sceneslam_core PUBLIC opencv_core opencv_calib3d
  opencv_features2d opencv_imgcodecs opencv_imgproc)



enable_testing()
add_executable(test_geometry tests/test_geometry.cpp)
target_include_directories(test_geometry PRIVATE include tests)
target_link_libraries(test_geometry PRIVATE sceneslam_core)
add_test(NAME geometry COMMAND test_geometry)
add_executable(test_association tests/test_association.cpp)
target_include_directories(test_association PRIVATE include tests)
target_link_libraries(test_association PRIVATE sceneslam_core opencv_imgcodecs)
add_test(NAME association COMMAND test_association)
add_executable(test_pnp tests/test_pnp.cpp)
target_include_directories(test_pnp PRIVATE include tests)
target_link_libraries(test_pnp PRIVATE sceneslam_core)
add_test(NAME pnp COMMAND test_pnp)
add_executable(test_outputs tests/test_outputs.cpp)
target_include_directories(test_outputs PRIVATE include tests)
target_link_libraries(test_outputs PRIVATE sceneslam_core)
add_test(NAME outputs COMMAND test_outputs)
add_executable(test_pipeline tests/test_pipeline.cpp)
target_include_directories(test_pipeline PRIVATE include tests)
target_link_libraries(test_pipeline PRIVATE sceneslam_core)
add_test(NAME pipeline COMMAND test_pipeline)

add_executable(scene_slam src/main.cpp)
target_link_libraries(scene_slam PRIVATE sceneslam_core)
add_test(NAME selftest COMMAND scene_slam selftest)
add_test(NAME missing_dataset
  COMMAND scene_slam run --dataset ${CMAKE_SOURCE_DIR}/missing-dataset
          --camera ${CMAKE_SOURCE_DIR}/configs/tum_freiburg1.yaml
          --out ${CMAKE_BINARY_DIR}/missing-output)
set_tests_properties(missing_dataset PROPERTIES WILL_FAIL TRUE)
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME scripts
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_scripts.py)
endif()
if(WIN32)
  set_tests_properties(geometry association pnp outputs pipeline selftest missing_dataset
    PROPERTIES ENVIRONMENT_MODIFICATION
    "PATH=path_list_prepend:${OpenCV_DIR}/x64/vc16/bin")
endif()
