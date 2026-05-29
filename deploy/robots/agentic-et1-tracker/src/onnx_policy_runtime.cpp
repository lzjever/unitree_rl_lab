#include "agentic_et1_tracker/policy/onnx_policy_runtime.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace agentic_et1_tracker {
namespace {

PolicyRuntimeError error(const std::string& message) {
  return PolicyRuntimeError("policy runtime error: " + message);
}

bool isGitLfsPointer(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  std::string prefix(128, '\0');
  in.read(prefix.data(), static_cast<std::streamsize>(prefix.size()));
  prefix.resize(static_cast<std::size_t>(in.gcount()));
  return prefix.rfind("version https://git-lfs.github.com/spec/v1", 0) == 0;
}

void validateModelFile(const std::filesystem::path& path) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    throw error("model file missing");
  }
  if (!std::filesystem::is_regular_file(path, ec) || ec) {
    throw error("model file is not regular");
  }
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    throw error("model file size unreadable");
  }
  if (size == 0) {
    throw error("model file empty");
  }
  if (isGitLfsPointer(path)) {
    throw error("model file is a Git LFS pointer");
  }
}

PolicyTensorElementType toPolicyTensorElementType(ONNXTensorElementDataType type) {
  switch (type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
      return PolicyTensorElementType::Float32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
      return PolicyTensorElementType::Float64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
      return PolicyTensorElementType::Int64;
    default:
      return PolicyTensorElementType::Unknown;
  }
}

PolicyTensorMetadata inputMetadata(const Ort::Session& session,
                                   Ort::AllocatorWithDefaultOptions& allocator,
                                   std::size_t index) {
  Ort::AllocatedStringPtr name = session.GetInputNameAllocated(index, allocator);
  Ort::TypeInfo type_info = session.GetInputTypeInfo(index);
  Ort::ConstTensorTypeAndShapeInfo tensor_info =
      type_info.GetTensorTypeAndShapeInfo();

  return {name.get(),
          toPolicyTensorElementType(tensor_info.GetElementType()),
          tensor_info.GetShape()};
}

PolicyTensorMetadata outputMetadata(const Ort::Session& session,
                                    Ort::AllocatorWithDefaultOptions& allocator,
                                    std::size_t index) {
  Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(index, allocator);
  Ort::TypeInfo type_info = session.GetOutputTypeInfo(index);
  Ort::ConstTensorTypeAndShapeInfo tensor_info =
      type_info.GetTensorTypeAndShapeInfo();

  return {name.get(),
          toPolicyTensorElementType(tensor_info.GetElementType()),
          tensor_info.GetShape()};
}

PolicyModelMetadata readMetadata(const Ort::Session& session,
                                 Ort::AllocatorWithDefaultOptions& allocator) {
  PolicyModelMetadata metadata;

  const std::size_t input_count = session.GetInputCount();
  metadata.inputs.reserve(input_count);
  for (std::size_t i = 0; i < input_count; ++i) {
    metadata.inputs.push_back(inputMetadata(session, allocator, i));
  }

  const std::size_t output_count = session.GetOutputCount();
  metadata.outputs.reserve(output_count);
  for (std::size_t i = 0; i < output_count; ++i) {
    metadata.outputs.push_back(outputMetadata(session, allocator, i));
  }

  return metadata;
}

void requireOutputTensor(const Ort::Value& value) {
  if (!value.IsTensor()) {
    throw error("actions output is not a tensor");
  }

  Ort::TensorTypeAndShapeInfo info = value.GetTensorTypeAndShapeInfo();
  if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
    throw error("actions output dtype is not float32");
  }
  if (info.GetElementCount() != kGaPolicyJointDim) {
    std::ostringstream out;
    out << "actions output size must be " << kGaPolicyJointDim << ", got "
        << info.GetElementCount();
    throw error(out.str());
  }
}

void requireFiniteActions(const Vec& actions) {
  const auto invalid = std::find_if(actions.begin(), actions.end(), [](float v) {
    return !std::isfinite(v);
  });
  if (invalid != actions.end()) {
    throw error("actions output contains non-finite values");
  }
}

}  // namespace

struct OnnxPolicyRuntime::Impl {
  Impl(const std::filesystem::path& model_path, const DeployConfig& deploy_config)
      : env(ORT_LOGGING_LEVEL_WARNING, "agentic_et1_policy_runtime") {
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session = std::make_unique<Ort::Session>(
        env, model_path.c_str(), session_options);

    const PolicyModelMetadata metadata = readMetadata(*session, allocator);
    validateGaPolicyIoContract(deploy_config, metadata);
  }

  Ort::Env env;
  Ort::SessionOptions session_options;
  Ort::AllocatorWithDefaultOptions allocator;
  std::unique_ptr<Ort::Session> session;
};

OnnxPolicyRuntime::OnnxPolicyRuntime(OnnxPolicyRuntimeConfig config) {
  try {
    validateModelFile(config.model_path);
    validateGaDeployConfig(config.deploy_config);
    impl_ = std::make_unique<Impl>(config.model_path, config.deploy_config);
  } catch (const PolicyRuntimeError&) {
    throw;
  } catch (const PolicyIoContractError& err) {
    throw error(err.what());
  } catch (const Ort::Exception& err) {
    throw error(std::string("ORT session init failed: ") + err.what());
  } catch (const std::exception& err) {
    throw error(std::string("init failed: ") + err.what());
  }
}

OnnxPolicyRuntime::~OnnxPolicyRuntime() = default;
OnnxPolicyRuntime::OnnxPolicyRuntime(OnnxPolicyRuntime&&) noexcept = default;
OnnxPolicyRuntime& OnnxPolicyRuntime::operator=(OnnxPolicyRuntime&&) noexcept =
    default;

Vec OnnxPolicyRuntime::infer(const PolicyInputs& inputs) {
  try {
    validateGaPolicyInputs(inputs);

    Vec obs_current = inputs.obs_current;
    Vec obs_history = inputs.obs_history;

    const std::array<std::int64_t, 2> obs_current_shape{
        1, static_cast<std::int64_t>(kGaPolicyObsCurrentDim)};
    const std::array<std::int64_t, 3> obs_history_shape{
        1, static_cast<std::int64_t>(kGaPolicyObsHistoryLength),
        static_cast<std::int64_t>(kGaPolicyObsHistoryWidth)};

    Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::array<Ort::Value, 2> input_tensors{
        Ort::Value::CreateTensor<float>(
            memory_info, obs_current.data(), obs_current.size(),
            obs_current_shape.data(), obs_current_shape.size()),
        Ort::Value::CreateTensor<float>(
            memory_info, obs_history.data(), obs_history.size(),
            obs_history_shape.data(), obs_history_shape.size()),
    };

    const std::array<const char*, 2> input_names{"obs_current", "obs_history"};
    const std::array<const char*, 1> output_names{"actions"};

    std::vector<Ort::Value> outputs = impl_->session->Run(
        Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(),
        input_tensors.size(), output_names.data(), output_names.size());

    if (outputs.size() != 1) {
      throw error("ORT returned wrong output count");
    }
    requireOutputTensor(outputs[0]);

    const float* data = outputs[0].GetTensorData<float>();
    Vec actions(data, data + kGaPolicyJointDim);
    requireFiniteActions(actions);
    return actions;
  } catch (const PolicyRuntimeError&) {
    throw;
  } catch (const PolicyIoContractError& err) {
    throw error(err.what());
  } catch (const Ort::Exception& err) {
    throw error(std::string("ORT run failed: ") + err.what());
  } catch (const std::exception& err) {
    throw error(std::string("infer failed: ") + err.what());
  }
}

}  // namespace agentic_et1_tracker
