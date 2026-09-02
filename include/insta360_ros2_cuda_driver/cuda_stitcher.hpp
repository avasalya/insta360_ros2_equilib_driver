#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace insta360_ros2_cuda_driver
{

struct StitchParameters
{
  int source_width;
  int source_height;
  int output_width;
  int output_height;
  int crop_size;
  float cx_offset;
  float cy_offset;
  float translation_x;
  float translation_y;
  float translation_z;
  float rotation_roll_deg;
  float rotation_pitch_deg;
  float rotation_yaw_deg;
  float fisheye_fov_deg;
  int cuda_device;
};

class CudaStitcher;

CudaStitcher * create_cuda_stitcher(
  const StitchParameters & parameters, std::string & error);

void destroy_cuda_stitcher(CudaStitcher * stitcher);

bool run_cuda_stitcher(
  CudaStitcher * stitcher,
  const std::uint8_t * source_bgr,
  std::size_t source_step,
  float & elapsed_ms,
  std::string & error);

const std::uint8_t * cuda_stitcher_output(const CudaStitcher * stitcher);

std::size_t cuda_stitcher_output_size(const CudaStitcher * stitcher);

}  // namespace insta360_ros2_cuda_driver

