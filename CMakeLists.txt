cmake_minimum_required(VERSION 3.8)
project(insta360_ros2_equilib_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-O3 -march=native -ffast-math -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

set(ENV{PKG_CONFIG_PATH}
  "$ENV{CONDA_PREFIX}/lib/pkgconfig:$ENV{CONDA_PREFIX}/share/pkgconfig:$ENV{PKG_CONFIG_PATH}"
)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(insta360_camera src/main.cpp)
target_link_libraries(insta360_camera
  CameraSDK ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES}
)
ament_target_dependencies(insta360_camera
  rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs std_srvs OpenCV
)

add_executable(decoder src/decoder.cpp)
target_link_libraries(decoder
  CameraSDK ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES}
)
ament_target_dependencies(decoder
  rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs std_srvs OpenCV
)

install(TARGETS insta360_camera decoder DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/equilib_stitcher.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch lib DESTINATION share/${PROJECT_NAME})

ament_package()
