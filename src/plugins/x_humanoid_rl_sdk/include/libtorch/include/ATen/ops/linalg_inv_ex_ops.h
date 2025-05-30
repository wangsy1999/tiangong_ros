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


struct TORCH_API linalg_inv_ex {
  using schema = ::std::tuple<at::Tensor,at::Tensor> (const at::Tensor &, bool);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::linalg_inv_ex")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "linalg_inv_ex(Tensor self, *, bool check_errors=False) -> (Tensor inverse, Tensor info)")
  static ::std::tuple<at::Tensor,at::Tensor> call(const at::Tensor & self, bool check_errors);
  static ::std::tuple<at::Tensor,at::Tensor> redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, bool check_errors);
};

struct TORCH_API linalg_inv_ex_inverse {
  using schema = ::std::tuple<at::Tensor &,at::Tensor &> (const at::Tensor &, bool, at::Tensor &, at::Tensor &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::linalg_inv_ex")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "inverse")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "linalg_inv_ex.inverse(Tensor self, *, bool check_errors=False, Tensor(a!) inverse, Tensor(b!) info) -> (Tensor(a!) inverse, Tensor(b!) info)")
  static ::std::tuple<at::Tensor &,at::Tensor &> call(const at::Tensor & self, bool check_errors, at::Tensor & inverse, at::Tensor & info);
  static ::std::tuple<at::Tensor &,at::Tensor &> redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, bool check_errors, at::Tensor & inverse, at::Tensor & info);
};

}} // namespace at::_ops
