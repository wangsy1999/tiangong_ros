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

namespace cuda {

TORCH_API ::std::tuple<at::Tensor,at::Tensor> _native_multi_head_attention(const at::Tensor & query, const at::Tensor & key, const at::Tensor & value, int64_t embed_dim, int64_t num_head, const at::Tensor & qkv_weight, const at::Tensor & qkv_bias, const at::Tensor & proj_weight, const at::Tensor & proj_bias, const c10::optional<at::Tensor> & mask={}, bool need_weights=true, bool average_attn_weights=true);

} // namespace cuda
} // namespace at
