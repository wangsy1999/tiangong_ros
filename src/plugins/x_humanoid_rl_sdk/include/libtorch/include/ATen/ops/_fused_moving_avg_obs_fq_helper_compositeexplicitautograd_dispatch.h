#pragma once
// @generated by torchgen/gen.py from DispatchKeyFunction.h

// NB: The implementing C++ file is RegisterDispatchKey.cpp

// The only #includes we need are for custom classes that have defaults in the C++ API
#include <c10/core/MemoryFormat.h>
#include <c10/core/Scalar.h>
#include <ATen/core/Reduction.h>

// Forward declarations of any types needed in the operator signatures.
// We can't directly include these classes because it will cause circular include dependencies.
// This file is included by TensorBody.h, which defines the Tensor class.
#include <ATen/core/ATen_fwd.h>

namespace at {

namespace compositeexplicitautograd {

TORCH_API ::std::tuple<at::Tensor,at::Tensor,at::Tensor,at::Tensor,at::Tensor,at::Tensor> _fused_moving_avg_obs_fq_helper_functional(const at::Tensor & self, const at::Tensor & observer_on, const at::Tensor & fake_quant_on, const at::Tensor & running_min, const at::Tensor & running_max, const at::Tensor & scale, const at::Tensor & zero_point, double averaging_const, int64_t quant_min, int64_t quant_max, int64_t ch_axis, bool per_row_fake_quant=false, bool symmetric_quant=false);

} // namespace compositeexplicitautograd
} // namespace at
