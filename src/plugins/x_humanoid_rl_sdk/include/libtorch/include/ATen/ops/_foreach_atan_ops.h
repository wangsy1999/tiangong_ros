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


struct TORCH_API _foreach_atan {
  using schema = ::std::vector<at::Tensor> (at::TensorList);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_foreach_atan")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_foreach_atan(Tensor[] tensors) -> Tensor[]")
  static ::std::vector<at::Tensor> call(at::TensorList tensors);
  static ::std::vector<at::Tensor> redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList tensors);
};

struct TORCH_API _foreach_atan_ {
  using schema = void (at::TensorList);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_foreach_atan_")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_foreach_atan_(Tensor(a!)[] self) -> ()")
  static void call(at::TensorList self);
  static void redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList self);
};

struct TORCH_API _foreach_atan_out {
  using schema = void (at::TensorList, at::TensorList);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_foreach_atan")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "out")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_foreach_atan.out(Tensor[] self, *, Tensor(a!)[] out) -> ()")
  static void call(at::TensorList self, at::TensorList out);
  static void redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList self, at::TensorList out);
};

struct TORCH_API _foreach_atan_functional {
  using schema = ::std::vector<at::Tensor> (at::TensorList);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_foreach_atan")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "functional")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_foreach_atan.functional(Tensor[] self) -> Tensor[] self_out")
  static ::std::vector<at::Tensor> call(at::TensorList self);
  static ::std::vector<at::Tensor> redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList self);
};

}} // namespace at::_ops
