// gpu_jpeg_encoder.hpp — JPEG encoding (and optionally Bayer demosaicing) on the GPU.
//
// Per-frame host work in the camera node is demosaic + JPEG encode, and with
// 14 cameras x 30 Hz x 1920x1200 that is the rig's CPU bottleneck: JPEG alone is
// ~12 ms per frame per core. This moves both to the GPU (NPP CFAToRGB + nvJPEG),
// leaving the CPU a 2.3 MB upload per frame.
//
// The header deliberately includes no CUDA headers (pimpl) so the node compiles
// unchanged when the CUDA libraries are absent; FLIR_HAVE_GPU_JPEG tells the node
// whether gpu_jpeg_encoder.cpp was built in.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace flir_gpu
{

enum class BayerPattern { kRGGB, kBGGR, kGRBG, kGBRG };

class GpuJpegEncoder
{
public:
  GpuJpegEncoder();
  ~GpuJpegEncoder();
  GpuJpegEncoder(const GpuJpegEncoder &) = delete;
  GpuJpegEncoder & operator=(const GpuJpegEncoder &) = delete;

  // Creates the CUDA stream, nvJPEG handle/state and NPP context on `device`.
  bool Init(int device, int quality, std::string * error);

  // 8-bit Bayer mosaic (host memory) -> demosaic on GPU -> JPEG.
  bool EncodeBayer8(
    const std::uint8_t * data, int width, int height, int stride,
    BayerPattern pattern, std::vector<std::uint8_t> & jpeg, std::string * error);

  // Interleaved RGB8 (host memory) -> JPEG.
  bool EncodeRgb8(
    const std::uint8_t * data, int width, int height, int stride,
    std::vector<std::uint8_t> & jpeg, std::string * error);

  // "NVIDIA GeForce RTX 3080 Ti (sm 8.6)"
  std::string Describe() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flir_gpu
