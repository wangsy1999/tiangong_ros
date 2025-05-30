#pragma once

// @generated by torchgen/gen.py from Operator.h

#include <tuple>
#include <vector>

// Forward declarations of any types needed in the operator signatures.
// We can't directly include these classes because it will cause circular include dependencies.
// This file is included by TensorBody.h, which defines the Tensor class.
#include <ATen/core/ATen_fwd.h>

namespace at {
namespace _ops {


struct TORCH_API slow_conv3d_forward_output {
  using schema = at::Tensor & (const at::Tensor &, const at::Tensor &, at::IntArrayRef, const c10::optional<at::Tensor> &, at::IntArrayRef, at::IntArrayRef, at::Tensor &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::slow_conv3d_forward")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "output")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "slow_conv3d_forward.output(Tensor self, Tensor weight, int[3] kernel_size, Tensor? bias, int[3] stride, int[3] padding, *, Tensor(a!) output) -> Tensor(a!)")
  static at::Tensor & call(const at::Tensor & self, const at::Tensor & weight, at::IntArrayRef kernel_size, const c10::optional<at::Tensor> & bias, at::IntArrayRef stride, at::IntArrayRef padding, at::Tensor & output);
  static at::Tensor & redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, const at::Tensor & weight, at::IntArrayRef kernel_size, const c10::optional<at::Tensor> & bias, at::IntArrayRef stride, at::IntArrayRef padding, at::Tensor & output);
};

struct TORCH_API slow_conv3d_forward {
  using schema = at::Tensor (const at::Tensor &, const at::Tensor &, at::IntArrayRef, const c10::optional<at::Tensor> &, at::IntArrayRef, at::IntArrayRef);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::slow_conv3d_forward")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "slow_conv3d_forward(Tensor self, Tensor weight, int[3] kernel_size, Tensor? bias, int[3] stride, int[3] padding) -> Tensor")
  static at::Tensor call(const at::Tensor & self, const at::Tensor & weight, at::IntArrayRef kernel_size, const c10::optional<at::Tensor> & bias, at::IntArrayRef stride, at::IntArrayRef padding);
  static at::Tensor redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, const at::Tensor & weight, at::IntArrayRef kernel_size, const c10::optional<at::Tensor> & bias, at::IntArrayRef stride, at::IntArrayRef padding);
};

}} // namespace at::_ops
