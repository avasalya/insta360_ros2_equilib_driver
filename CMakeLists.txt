cmake_minimum_required(VERSION 3.8)
project(insta360_ros2_equilib_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-O3>
    $<$<COMPILE_LANGUAGE:CXX>:-march=native>
    $<$<COMPILE_LANGUAGE:CXX>:-ffast-math>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

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

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
  CameraSDK ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs std_srvs OpenCV
)

add_executable(decoder src/decoder.cpp)
target_link_libraries(decoder
  CameraSDK ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES} CUDA::nvToolsExt
)
ament_target_dependencies(decoder
  rclcpp camera_info_manager cv_bridge image_transport sensor_msgs std_msgs std_srvs OpenCV
)

add_executable(cuda_stitcher
  src/cuda_stitcher_node.cpp
  src/cuda_stitcher.cu
)
target_include_directories(cuda_stitcher PRIVATE include)
target_link_libraries(cuda_stitcher CUDA::cudart CUDA::nvToolsExt)
target_compile_features(cuda_stitcher PRIVATE cxx_std_17)
target_compile_options(cuda_stitcher PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:-O3>
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
)
set_target_properties(cuda_stitcher PROPERTIES
  CUDA_ARCHITECTURES 75
  CUDA_RUNTIME_LIBRARY Shared
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
)
ament_target_dependencies(cuda_stitcher rclcpp sensor_msgs)

install(TARGETS ${PROJECT_NAME} decoder cuda_stitcher DESTINATION lib/${PROJECT_NAME})

# Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/equilib_stitcher.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch lib DESTINATION share/${PROJECT_NAME})


ament_package()
