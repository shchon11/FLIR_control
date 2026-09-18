// gpu_jpeg_encoder.cpp — see gpu_jpeg_encoder.hpp.
//
// Only CUDA runtime, NPP and nvJPEG *library* calls — no custom kernels, so no
// nvcc is needed; the libraries can come from NVIDIA's pip wheels (see CMakeLists).

#include "gpu_jpeg_encoder.hpp"

#include <cuda_runtime_api.h>
#include <nppcore.h>
#include <nppi_color_conversion.h>
#include <nvjpeg.h>

#include <sstream>

namespace flir_gpu
{

namespace
{

bool CudaOk(cudaError_t status, const char * what, std::string * error)
{
  if (status == cudaSuccess) {
    return true;
  }
  if (error) {
    *error = std::string(what) + ": " + cudaGetErrorString(status);
  }
  return false;
}

bool NvjpegOk(nvjpegStatus_t status, const char * what, std::string * error)
{
  if (status == NVJPEG_STATUS_SUCCESS) {
    return true;
  }
  if (error) {
    *error = std::string(what) + ": nvJPEG status " + std::to_string(static_cast<int>(status));
  }
  return false;
}

bool NppOk(NppStatus status, const char * what, std::string * error)
{
  if (status >= NPP_NO_ERROR) {   // positive values are warnings
    return true;
  }
  if (error) {
    *error = std::string(what) + ": NPP status " + std::to_string(static_cast<int>(status));
  }
  return false;
}

NppiBayerGridPosition GridOf(BayerPattern pattern)
{
  switch (pattern) {
    case BayerPattern::kBGGR: return NPPI_BAYER_BGGR;
    case BayerPattern::kGRBG: return NPPI_BAYER_GRBG;
    case BayerPattern::kGBRG: return NPPI_BAYER_GBRG;
    case BayerPattern::kRGGB:
    default: return NPPI_BAYER_RGGB;
  }
}

}  // namespace

struct GpuJpegEncoder::Impl
{
  int device = 0;
  cudaStream_t stream = nullptr;
  nvjpegHandle_t handle = nullptr;
  nvjpegEncoderState_t state = nullptr;
  nvjpegEncoderParams_t params = nullptr;
  NppStreamContext npp{};
  std::uint8_t * d_bayer = nullptr;
  std::size_t bayer_bytes = 0;
  std::uint8_t * d_rgb = nullptr;
  std::size_t rgb_bytes = 0;
  std::string name;

  ~Impl()
  {
    if (params) {nvjpegEncoderParamsDestroy(params);}
    if (state) {nvjpegEncoderStateDestroy(state);}
    if (handle) {nvjpegDestroy(handle);}
    if (d_bayer) {cudaFree(d_bayer);}
    if (d_rgb) {cudaFree(d_rgb);}
    if (stream) {cudaStreamDestroy(stream);}
  }

  // Device buffers are sized on the first frame and reused (resolution rarely changes).
  bool Reserve(std::uint8_t *& buffer, std::size_t & have, std::size_t need, std::string * error)
  {
    if (have >= need) {
      return true;
    }
    if (buffer) {
      cudaFree(buffer);
      buffer = nullptr;
      have = 0;
    }
    if (!CudaOk(cudaMalloc(reinterpret_cast<void **>(&buffer), need), "cudaMalloc", error)) {
      return false;
    }
    have = need;
    return true;
  }

  bool EncodeDeviceRgb(int width, int height, std::vector<std::uint8_t> & jpeg, std::string * error)
  {
    nvjpegImage_t image{};
    image.channel[0] = d_rgb;
    image.pitch[0] = static_cast<std::size_t>(width) * 3U;
    if (!NvjpegOk(
        nvjpegEncodeImage(handle, state, params, &image, NVJPEG_INPUT_RGBI, width, height, stream),
        "nvjpegEncodeImage", error))
    {
      return false;
    }
    std::size_t length = 0;
    if (!NvjpegOk(
        nvjpegEncodeRetrieveBitstream(handle, state, nullptr, &length, stream),
        "nvjpegEncodeRetrieveBitstream(size)", error) ||
      !CudaOk(cudaStreamSynchronize(stream), "cudaStreamSynchronize", error))
    {
      return false;
    }
    jpeg.resize(length);
    if (!NvjpegOk(
        nvjpegEncodeRetrieveBitstream(handle, state, jpeg.data(), &length, stream),
        "nvjpegEncodeRetrieveBitstream", error) ||
      !CudaOk(cudaStreamSynchronize(stream), "cudaStreamSynchronize", error))
    {
      return false;
    }
    jpeg.resize(length);
    return true;
  }
};

GpuJpegEncoder::GpuJpegEncoder()
: impl_(std::make_unique<Impl>())
{
}

GpuJpegEncoder::~GpuJpegEncoder() = default;

bool GpuJpegEncoder::Init(int device, int quality, std::string * error)
{
  Impl & d = *impl_;
  d.device = device;
  if (!CudaOk(cudaSetDevice(device), "cudaSetDevice", error) ||
    !CudaOk(cudaStreamCreateWithFlags(&d.stream, cudaStreamNonBlocking), "cudaStreamCreate", error))
  {
    return false;
  }

  cudaDeviceProp prop{};
  if (cudaGetDeviceProperties(&prop, device) == cudaSuccess) {
    std::ostringstream oss;
    oss << prop.name << " (sm " << prop.major << "." << prop.minor << ")";
    d.name = oss.str();
  }

  // NPP needs the stream plus device facts; nppGetStreamContext fills the facts for
  // the current device, then we point it at our stream.
  if (!NppOk(nppGetStreamContext(&d.npp), "nppGetStreamContext", error)) {
    return false;
  }
  d.npp.hStream = d.stream;
  unsigned int flags = 0;
  cudaStreamGetFlags(d.stream, &flags);
  d.npp.nStreamFlags = flags;

  if (!NvjpegOk(nvjpegCreateSimple(&d.handle), "nvjpegCreateSimple", error) ||
    !NvjpegOk(nvjpegEncoderStateCreate(d.handle, &d.state, d.stream), "nvjpegEncoderStateCreate", error) ||
    !NvjpegOk(nvjpegEncoderParamsCreate(d.handle, &d.params, d.stream), "nvjpegEncoderParamsCreate", error) ||
    !NvjpegOk(nvjpegEncoderParamsSetQuality(d.params, quality, d.stream), "SetQuality", error) ||
    // 4:2:0 like OpenCV's default, so file sizes and look stay comparable to the CPU path
    !NvjpegOk(nvjpegEncoderParamsSetSamplingFactors(d.params, NVJPEG_CSS_420, d.stream), "SetSampling", error) ||
    !NvjpegOk(nvjpegEncoderParamsSetOptimizedHuffman(d.params, 0, d.stream), "SetOptimizedHuffman", error))
  {
    return false;
  }
  return true;
}

bool GpuJpegEncoder::EncodeBayer8(
  const std::uint8_t * data, int width, int height, int stride,
  BayerPattern pattern, std::vector<std::uint8_t> & jpeg, std::string * error)
{
  Impl & d = *impl_;
  const std::size_t bayer = static_cast<std::size_t>(width) * height;
  const std::size_t rgb = bayer * 3U;
  if (!d.Reserve(d.d_bayer, d.bayer_bytes, bayer, error) ||
    !d.Reserve(d.d_rgb, d.rgb_bytes, rgb, error))
  {
    return false;
  }
  if (!CudaOk(
      cudaMemcpy2DAsync(
        d.d_bayer, width, data, stride, width, height, cudaMemcpyHostToDevice, d.stream),
      "upload bayer", error))
  {
    return false;
  }
  const NppiSize size{width, height};
  const NppiRect roi{0, 0, width, height};
  if (!NppOk(
      nppiCFAToRGB_8u_C1C3R_Ctx(
        d.d_bayer, width, size, roi, d.d_rgb, width * 3, GridOf(pattern), NPPI_INTER_UNDEFINED, d.npp),
      "nppiCFAToRGB", error))
  {
    return false;
  }
  return d.EncodeDeviceRgb(width, height, jpeg, error);
}

bool GpuJpegEncoder::EncodeRgb8(
  const std::uint8_t * data, int width, int height, int stride,
  std::vector<std::uint8_t> & jpeg, std::string * error)
{
  Impl & d = *impl_;
  const std::size_t row = static_cast<std::size_t>(width) * 3U;
  if (!d.Reserve(d.d_rgb, d.rgb_bytes, row * height, error)) {
    return false;
  }
  if (!CudaOk(
      cudaMemcpy2DAsync(d.d_rgb, row, data, stride, row, height, cudaMemcpyHostToDevice, d.stream),
      "upload rgb", error))
  {
    return false;
  }
  return d.EncodeDeviceRgb(width, height, jpeg, error);
}

std::string GpuJpegEncoder::Describe() const
{
  return impl_->name;
}

}  // namespace flir_gpu
