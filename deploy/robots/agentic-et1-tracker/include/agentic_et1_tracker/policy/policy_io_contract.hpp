#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "agentic_et1_tracker/policy/policy_math.hpp"

namespace agentic_et1_tracker {

inline constexpr std::size_t kGaPolicyJointDim = 26;
inline constexpr std::size_t kGaPolicyObsCurrentDim = 131;
inline constexpr std::size_t kGaPolicyObsHistoryLength = 25;
inline constexpr std::size_t kGaPolicyObsHistoryWidth = 105;

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
void validateGaPolicyInputs(const PolicyInputs& inputs);

}  // namespace agentic_et1_tracker
