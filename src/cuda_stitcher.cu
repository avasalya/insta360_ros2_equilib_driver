#include "insta360_ros2_cuda_driver/cuda_stitcher.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <new>
#include <string>
#include <vector>

namespace insta360_ros2_cuda_driver
{

namespace
{

constexpr float kPi = 3.14159265358979323846F;

std::string cuda_error(const char * operation, cudaError_t result)
{
  return std::string(operation) + ": " + cudaGetErrorString(result);
}

std::array<float, 9> rotation_matrix(float roll, float pitch, float yaw)
{
  const float cr = std::cos(roll);
  const float sr = std::sin(roll);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);

  // Rz * Ry * Rx, matching the Python implementation.
  return {
    cy * cp,
    cy * sp * sr - sy * cr,
    cy * sp * cr + sy * sr,
    sy * cp,
    sy * sp * sr + cy * cr,
    sy * sp * cr - cy * sr,
    -sp,
    cp * sr,
    cp * cr};
}

std::vector<float2> build_source_map(const StitchParameters & p)
{
  const int lens_width = p.source_width / 2;
  const int y_offset = (p.source_height - p.crop_size) / 2;
  const int x_offset = (lens_width - p.crop_size) / 2;
  const float half_fov = p.fisheye_fov_deg * kPi / 360.0F;
  const float cx = static_cast<float>(p.crop_size) * 0.5F + p.cx_offset;
  const float cy = static_cast<float>(p.crop_size) * 0.5F + p.cy_offset;
  const auto rotation = rotation_matrix(
    p.rotation_roll_deg * kPi / 180.0F,
    p.rotation_pitch_deg * kPi / 180.0F,
    p.rotation_yaw_deg * kPi / 180.0F);

  std::vector<float2> map(
    static_cast<std::size_t>(p.output_width) * static_cast<std::size_t>(p.output_height));

  for (int y = 0; y < p.output_height; ++y) {
    const float latitude =
      (static_cast<float>(y) / static_cast<float>(p.output_height)) * kPi - kPi * 0.5F;
    const float cos_lat = std::cos(latitude);
    const float sin_lat = std::sin(latitude);

    for (int x = 0; x < p.output_width; ++x) {
      const float longitude =
        (static_cast<float>(x) / static_cast<float>(p.output_width)) * (2.0F * kPi) - kPi;
      const float orig_x = cos_lat * std::sin(longitude);
      const float orig_y = sin_lat;
      const float x_value = orig_y;
      const float y_value = -orig_x;
      const float z_value = cos_lat * std::cos(longitude);
      const bool front = z_value >= 0.0F;

      const float transformed_x =
        rotation[0] * x_value + rotation[1] * y_value + rotation[2] * z_value +
        p.translation_x;
      const float transformed_y =
        rotation[3] * x_value + rotation[4] * y_value + rotation[5] * z_value +
        p.translation_y;
      const float transformed_z =
        rotation[6] * x_value + rotation[7] * y_value + rotation[8] * z_value +
        p.translation_z;

      const float final_x = front ? x_value : -transformed_x;
      const float final_y = front ? y_value : transformed_y;
      const float final_z = front ? z_value : transformed_z;
      const float radius = std::max(std::sqrt(final_x * final_x + final_y * final_y), 1.0e-6F);
      const float theta = std::atan2(radius, std::abs(final_z));
      const float fisheye_radius =
        (theta / half_fov) * (static_cast<float>(p.crop_size) * 0.5F);
      const float u = cx + (final_x / radius) * fisheye_radius;
      const float v = cy + (final_y / radius) * fisheye_radius;

      const float rotated_u = front ? (static_cast<float>(p.crop_size) - 1.0F) - v : v;
      const float rotated_v = front ? u : (static_cast<float>(p.crop_size) - 1.0F) - u;
      const float source_x =
        front ? static_cast<float>(lens_width + x_offset) + rotated_u :
        static_cast<float>(x_offset) + rotated_u;
      const float source_y = static_cast<float>(y_offset) + rotated_v;

      map[static_cast<std::size_t>(y) * p.output_width + x] = make_float2(source_x, source_y);
    }
  }

  return map;
}

__device__ inline float sample_channel(
  const std::uint8_t * source,
  int width,
  int height,
  int x,
  int y,
  int channel)
{
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return 0.0F;
  }
  return static_cast<float>(source[(y * width + x) * 3 + channel]);
}

__global__ void remap_bilinear_bgr(
  const std::uint8_t * source,
  int source_width,
  int source_height,
  std::uint8_t * output,
  int output_width,
  int output_height,
  const float2 * map)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= output_width || y >= output_height) {
    return;
  }

  const std::size_t output_index =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(output_width) +
    static_cast<std::size_t>(x);
  const float2 coordinate = map[output_index];
  const int x0 = static_cast<int>(floorf(coordinate.x));
  const int y0 = static_cast<int>(floorf(coordinate.y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const float wx = coordinate.x - static_cast<float>(x0);
  const float wy = coordinate.y - static_cast<float>(y0);

  for (int channel = 0; channel < 3; ++channel) {
    const float p00 = sample_channel(source, source_width, source_height, x0, y0, channel);
    const float p10 = sample_channel(source, source_width, source_height, x1, y0, channel);
    const float p01 = sample_channel(source, source_width, source_height, x0, y1, channel);
    const float p11 = sample_channel(source, source_width, source_height, x1, y1, channel);
    const float top = p00 + wx * (p10 - p00);
    const float bottom = p01 + wx * (p11 - p01);
    const float value = top + wy * (bottom - top);
    output[output_index * 3 + channel] =
      static_cast<std::uint8_t>(fminf(fmaxf(value, 0.0F), 255.0F) + 0.5F);
  }
}

}  // namespace

class CudaStitcher
{
public:
  StitchParameters parameters{};
  std::uint8_t * device_source{nullptr};
  std::uint8_t * device_output{nullptr};
  float2 * device_map{nullptr};
  std::uint8_t * host_output{nullptr};
  cudaStream_t stream{nullptr};
  cudaEvent_t started{nullptr};
  cudaEvent_t finished{nullptr};
  std::size_t source_row_bytes{0};
  std::size_t source_bytes{0};
  std::size_t output_bytes{0};
};

CudaStitcher * create_cuda_stitcher(
  const StitchParameters & parameters, std::string & error)
{
  if (
    parameters.source_width <= 0 || parameters.source_height <= 0 ||
    parameters.output_width <= 0 || parameters.output_height <= 0 ||
    parameters.source_width % 2 != 0)
  {
    error = "invalid source/output dimensions";
    return nullptr;
  }

  const int lens_width = parameters.source_width / 2;
  if (
    parameters.crop_size <= 0 ||
    parameters.crop_size > std::min(parameters.source_height, lens_width))
  {
    error = "crop_size is outside the fisheye lens dimensions";
    return nullptr;
  }

  auto * stitcher = new (std::nothrow) CudaStitcher();
  if (stitcher == nullptr) {
    error = "failed to allocate CudaStitcher";
    return nullptr;
  }

  stitcher->parameters = parameters;
  stitcher->source_row_bytes = static_cast<std::size_t>(parameters.source_width) * 3U;
  stitcher->source_bytes =
    stitcher->source_row_bytes * static_cast<std::size_t>(parameters.source_height);
  stitcher->output_bytes =
    static_cast<std::size_t>(parameters.output_width) *
    static_cast<std::size_t>(parameters.output_height) * 3U;

  auto fail = [&](const char * operation, cudaError_t result) {
      error = cuda_error(operation, result);
      destroy_cuda_stitcher(stitcher);
      return static_cast<CudaStitcher *>(nullptr);
    };

  cudaError_t result = cudaSetDevice(parameters.cuda_device);
  if (result != cudaSuccess) {
    return fail("cudaSetDevice", result);
  }
  result = cudaStreamCreateWithFlags(&stitcher->stream, cudaStreamNonBlocking);
  if (result != cudaSuccess) {
    return fail("cudaStreamCreateWithFlags", result);
  }
  result = cudaEventCreate(&stitcher->started);
  if (result != cudaSuccess) {
    return fail("cudaEventCreate(started)", result);
  }
  result = cudaEventCreate(&stitcher->finished);
  if (result != cudaSuccess) {
    return fail("cudaEventCreate(finished)", result);
  }
  result = cudaMalloc(
    reinterpret_cast<void **>(&stitcher->device_source), stitcher->source_bytes);
  if (result != cudaSuccess) {
    return fail("cudaMalloc(source)", result);
  }
  result = cudaMalloc(
    reinterpret_cast<void **>(&stitcher->device_output), stitcher->output_bytes);
  if (result != cudaSuccess) {
    return fail("cudaMalloc(output)", result);
  }
  result = cudaHostAlloc(
    reinterpret_cast<void **>(&stitcher->host_output),
    stitcher->output_bytes,
    cudaHostAllocPortable);
  if (result != cudaSuccess) {
    return fail("cudaHostAlloc(output)", result);
  }

  const auto host_map = build_source_map(parameters);
  const std::size_t map_bytes = host_map.size() * sizeof(float2);
  result = cudaMalloc(reinterpret_cast<void **>(&stitcher->device_map), map_bytes);
  if (result != cudaSuccess) {
    return fail("cudaMalloc(map)", result);
  }
  result = cudaMemcpy(
    stitcher->device_map, host_map.data(), map_bytes, cudaMemcpyHostToDevice);
  if (result != cudaSuccess) {
    return fail("cudaMemcpy(map)", result);
  }

  return stitcher;
}

void destroy_cuda_stitcher(CudaStitcher * stitcher)
{
  if (stitcher == nullptr) {
    return;
  }
  if (stitcher->stream != nullptr) {
    cudaStreamSynchronize(stitcher->stream);
  }
  if (stitcher->device_map != nullptr) {
    cudaFree(stitcher->device_map);
  }
  if (stitcher->device_output != nullptr) {
    cudaFree(stitcher->device_output);
  }
  if (stitcher->device_source != nullptr) {
    cudaFree(stitcher->device_source);
  }
  if (stitcher->host_output != nullptr) {
    cudaFreeHost(stitcher->host_output);
  }
  if (stitcher->started != nullptr) {
    cudaEventDestroy(stitcher->started);
  }
  if (stitcher->finished != nullptr) {
    cudaEventDestroy(stitcher->finished);
  }
  if (stitcher->stream != nullptr) {
    cudaStreamDestroy(stitcher->stream);
  }
  delete stitcher;
}

bool run_cuda_stitcher(
  CudaStitcher * stitcher,
  const std::uint8_t * source_bgr,
  std::size_t source_step,
  float & elapsed_ms,
  std::string & error)
{
  if (stitcher == nullptr || source_bgr == nullptr) {
    error = "stitcher/source is null";
    return false;
  }
  if (source_step < stitcher->source_row_bytes) {
    error = "source step is smaller than width * 3";
    return false;
  }

  cudaError_t result = cudaEventRecord(stitcher->started, stitcher->stream);
  if (result != cudaSuccess) {
    error = cuda_error("cudaEventRecord(started)", result);
    return false;
  }
  result = cudaMemcpy2DAsync(
    stitcher->device_source,
    stitcher->source_row_bytes,
    source_bgr,
    source_step,
    stitcher->source_row_bytes,
    stitcher->parameters.source_height,
    cudaMemcpyHostToDevice,
    stitcher->stream);
  if (result != cudaSuccess) {
    error = cuda_error("cudaMemcpy2DAsync(source)", result);
    return false;
  }

  const dim3 block(16, 16);
  const dim3 grid(
    (stitcher->parameters.output_width + block.x - 1) / block.x,
    (stitcher->parameters.output_height + block.y - 1) / block.y);
  remap_bilinear_bgr<<<grid, block, 0, stitcher->stream>>>(
    stitcher->device_source,
    stitcher->parameters.source_width,
    stitcher->parameters.source_height,
    stitcher->device_output,
    stitcher->parameters.output_width,
    stitcher->parameters.output_height,
    stitcher->device_map);
  result = cudaGetLastError();
  if (result != cudaSuccess) {
    error = cuda_error("remap_bilinear_bgr", result);
    return false;
  }

  result = cudaMemcpyAsync(
    stitcher->host_output,
    stitcher->device_output,
    stitcher->output_bytes,
    cudaMemcpyDeviceToHost,
    stitcher->stream);
  if (result != cudaSuccess) {
    error = cuda_error("cudaMemcpyAsync(output)", result);
    return false;
  }
  result = cudaEventRecord(stitcher->finished, stitcher->stream);
  if (result != cudaSuccess) {
    error = cuda_error("cudaEventRecord(finished)", result);
    return false;
  }
  result = cudaEventSynchronize(stitcher->finished);
  if (result != cudaSuccess) {
    error = cuda_error("cudaEventSynchronize(finished)", result);
    return false;
  }
  result = cudaEventElapsedTime(&elapsed_ms, stitcher->started, stitcher->finished);
  if (result != cudaSuccess) {
    error = cuda_error("cudaEventElapsedTime", result);
    return false;
  }
  return true;
}

const std::uint8_t * cuda_stitcher_output(const CudaStitcher * stitcher)
{
  return stitcher == nullptr ? nullptr : stitcher->host_output;
}

std::size_t cuda_stitcher_output_size(const CudaStitcher * stitcher)
{
  return stitcher == nullptr ? 0U : stitcher->output_bytes;
}

}  // namespace insta360_ros2_cuda_driver

