cmake_minimum_required(VERSION 3.20)
project(SceneSLAM VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d features2d imgcodecs imgproc)

add_library(sceneslam_core src/geometry.cpp src/rgbd.cpp src/odometry.cpp
  src/map.cpp src/output.cpp)

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
if(WIN32)
  set_tests_properties(geometry association pnp outputs PROPERTIES ENVIRONMENT_MODIFICATION
    "PATH=path_list_prepend:${OpenCV_DIR}/x64/vc16/bin")
endif()





