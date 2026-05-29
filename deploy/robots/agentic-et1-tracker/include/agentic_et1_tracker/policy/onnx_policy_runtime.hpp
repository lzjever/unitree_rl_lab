#pragma once

#include <filesystem>
#include <memory>
#include <stdexcept>

#include "agentic_et1_tracker/policy/policy_io_contract.hpp"
#include "agentic_et1_tracker/policy/policy_step_runner.hpp"

namespace agentic_et1_tracker {

struct OnnxPolicyRuntimeConfig {
  std::filesystem::path model_path;
  DeployConfig deploy_config;
};

class PolicyRuntimeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

class OnnxPolicyRuntime final : public PolicyInference {
 public:
  explicit OnnxPolicyRuntime(OnnxPolicyRuntimeConfig config);
  ~OnnxPolicyRuntime() override;

  OnnxPolicyRuntime(const OnnxPolicyRuntime&) = delete;
  OnnxPolicyRuntime& operator=(const OnnxPolicyRuntime&) = delete;
  OnnxPolicyRuntime(OnnxPolicyRuntime&&) noexcept;
  OnnxPolicyRuntime& operator=(OnnxPolicyRuntime&&) noexcept;

  Vec infer(const PolicyInputs& inputs) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace agentic_et1_tracker

