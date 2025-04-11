#include <NvInfer.h>

#include <iostream>

#include "absl/flags/flag.h"
#include "absl/log/log.h"

#include "aos/init.h"
#include "aos/util/file.h"
#include "frc/orin/cuda.h"

ABSL_FLAG(std::string, engine_path, "", "Path to the TensorRT engine to use.");

namespace yolo {

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char *msg) noexcept override {
    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        LOG(FATAL) << msg;
        break;
      case Severity::kERROR:
        LOG(ERROR) << msg;
        break;
      case Severity::kWARNING:
        LOG(WARNING) << msg;
        break;
      case Severity::kINFO:
        LOG(INFO) << msg;
        break;
      case Severity::kVERBOSE:
        VLOG(1) << msg;
        break;
    }
  }
};

class ModelInference {
 public:
  ModelInference(const std::string &engine_path)
      : engine_(nullptr), context_(nullptr) {
    InitializeEngine(engine_path);
  }

  ~ModelInference() {
    for (void *buf : device_buffers_) {
      cudaFree(buf);
    }
    if (stream_) cudaStreamDestroy(stream_);
  }

  bool Infer(const float *input, float *output) {
    // Copy input to device
    CHECK_CUDA(cudaMemcpyAsync(device_buffers_[0], input, input_size_,
                               cudaMemcpyHostToDevice, stream_));

    // Execute inference
    for (int i = 0; i < engine_->getNbIOTensors(); i++) {
      const char *tensor_name = engine_->getIOTensorName(i);
      context_->setTensorAddress(tensor_name, device_buffers_[i]);
    }

    if (!context_->enqueueV3(stream_)) {
      LOG(FATAL) << "Error running inference: enqueueV3 failed!" << std::endl;
    }

    // Copy output back to host
    CHECK_CUDA(cudaMemcpyAsync(output, device_buffers_[1], output_size_,
                               cudaMemcpyDeviceToHost, stream_));

    // Synchronize stream
    CHECK_CUDA(cudaStreamSynchronize(stream_));

    return true;
  }

  nvinfer1::Dims get_input_dims() const { return input_dims_; }
  nvinfer1::Dims get_output_dims() const { return output_dims_; }
  size_t get_input_size() const { return input_size_; }
  size_t get_output_size() const { return output_size_; }

 private:
  bool InitializeEngine(const std::string &engine_path) {
    // Read engine file
    std::string engine_data = aos::util::ReadFileToStringOrDie(engine_path);

    // Create runtime and engine
    runtime_ = std::unique_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      LOG(FATAL) << "Error creating TensorRT runtime" << std::endl;
      return false;
    }

    engine_ =
        std::unique_ptr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
            engine_data.data(), engine_data.size()));
    if (!engine_) {
      LOG(FATAL) << "Error deserializing CUDA engine" << std::endl;
      return false;
    }

    context_ = std::unique_ptr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext());
    if (!context_) {
      LOG(FATAL) << "Error creating execution context" << std::endl;
      return false;
    }

    // Create CUDA stream
    CHECK_CUDA(cudaStreamCreate(&stream_));

    // Allocate device buffers
    for (int i = 0; i < engine_->getNbIOTensors(); i++) {
      const char *tensor_name = engine_->getIOTensorName(i);
      nvinfer1::Dims dims = engine_->getTensorShape(tensor_name);
      size_t size = 1;
      for (int j = 0; j < dims.nbDims; j++) {
        size *= dims.d[j];
      }
      size *= sizeof(float);  // Assuming float32 data type

      void *deviceBuffer;
      CHECK_CUDA(cudaMalloc(&deviceBuffer, size));
      device_buffers_.push_back(deviceBuffer);

      if (engine_->getTensorIOMode(tensor_name) ==
          nvinfer1::TensorIOMode::kINPUT) {
        input_size_ = size;
        input_dims_ = dims;
      } else {
        output_size_ = size;
        output_dims_ = dims;
      }
    }

    return true;
  }

  Logger logger_;

  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_;
  std::vector<void *> device_buffers_;
  size_t input_size_;
  size_t output_size_;
  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
};

void Main() {
  ModelInference inference(absl::GetFlag(FLAGS_engine_path));
  LOG(INFO) << "Yup";
}

}  // namespace yolo

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  yolo::Main();
  return 0;
}
