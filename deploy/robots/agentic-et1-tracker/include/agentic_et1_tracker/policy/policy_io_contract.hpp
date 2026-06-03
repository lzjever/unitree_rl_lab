#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/policy_math.hpp"
#include "agentic_et1_tracker/policy/velocity_deploy_config.hpp"

namespace agentic_et1_tracker {

inline constexpr std::size_t kGaPolicyJointDim = 26;
inline constexpr std::size_t kGaPolicyObsCurrentDim = 131;
inline constexpr std::size_t kGaPolicyObsHistoryLength = 25;
inline constexpr std::size_t kGaPolicyObsHistoryWidth = 105;
inline constexpr std::size_t kClnPolicyObsCurrentDim = 121;
inline constexpr std::size_t kClnPolicyObsHistoryLength = 25;
inline constexpr std::size_t kClnPolicyObsHistoryWidth = 35;

enum class PolicyTensorElementType {
  Float32,
  Float64,
  Int64,
  Unknown,
};

struct PolicyTensorMetadata {
  std::string name;
  PolicyTensorElementType element_type{PolicyTensorElementType::Unknown};
  std::vector<std::int64_t> shape;
};

struct PolicyModelMetadata {
  std::vector<PolicyTensorMetadata> inputs;
  std::vector<PolicyTensorMetadata> outputs;
};

class PolicyIoContractError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

void validateGaDeployConfig(const DeployConfig& config);
void validateGaPolicyIoContract(const DeployConfig& config,
                                const PolicyModelMetadata& metadata);
void validateGaPolicyInputs(const DeployConfig& config, const PolicyInputs& inputs);
void validateVelocityDeployConfig(const VelocityDeployConfig& config);
void validateVelocityPolicyIoContract(const VelocityDeployConfig& config,
                                      const PolicyModelMetadata& metadata);
void validateVelocityPolicyInputs(const Vec& obs);

}  // namespace agentic_et1_tracker
