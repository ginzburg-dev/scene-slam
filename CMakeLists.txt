cmake_minimum_required(VERSION 3.20)
project(SceneSLAM VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d)

add_library(sceneslam_core src/geometry.cpp)
target_include_directories(sceneslam_core PUBLIC include)
target_link_libraries(sceneslam_core PUBLIC opencv_core opencv_calib3d)

enable_testing()
add_executable(test_geometry tests/test_geometry.cpp)
target_include_directories(test_geometry PRIVATE include tests)
target_link_libraries(test_geometry PRIVATE sceneslam_core)
add_test(NAME geometry COMMAND test_geometry)
if(WIN32)
  set_tests_properties(geometry PROPERTIES ENVIRONMENT_MODIFICATION
    "PATH=path_list_prepend:${OpenCV_DIR}/x64/vc16/bin")
endif()


