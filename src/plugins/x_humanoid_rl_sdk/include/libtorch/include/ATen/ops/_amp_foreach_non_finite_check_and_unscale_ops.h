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


struct TORCH_API _amp_foreach_non_finite_check_and_unscale_ {
  using schema = void (at::TensorList, at::Tensor &, const at::Tensor &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_amp_foreach_non_finite_check_and_unscale_")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_amp_foreach_non_finite_check_and_unscale_(Tensor(a!)[] self, Tensor(b!) found_inf, Tensor inv_scale) -> ()")
  static void call(at::TensorList self, at::Tensor & found_inf, const at::Tensor & inv_scale);
  static void redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList self, at::Tensor & found_inf, const at::Tensor & inv_scale);
};

struct TORCH_API _amp_foreach_non_finite_check_and_unscale_out {
  using schema = void (at::TensorList, at::Tensor &, const at::Tensor &, at::TensorList);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_amp_foreach_non_finite_check_and_unscale")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "out")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_amp_foreach_non_finite_check_and_unscale.out(Tensor[] self, Tensor(b!) found_inf, Tensor inv_scale, *, Tensor(a!)[] out) -> ()")
  static void call(at::TensorList self, at::Tensor & found_inf, const at::Tensor & inv_scale, at::TensorList out);
  static void redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList self, at::Tensor & found_inf, const at::Tensor & inv_scale, at::TensorList out);
};

struct TORCH_API _amp_foreach_non_finite_check_and_unscale_functional {
  using schema = ::std::tuple<::std::vector<at::Tensor>,at::Tensor> (at::TensorList, const at::Tensor &, const at::Tensor &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::_amp_foreach_non_finite_check_and_unscale")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "functional")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "_amp_foreach_non_finite_check_and_unscale.functional(Tensor[] self, Tensor found_inf, Tensor inv_scale) -> (Tensor[] self_out, Tensor found_inf_out)")
  static ::std::tuple<::std::vector<at::Tensor>,at::Tensor> call(at::TensorList self, const at::Tensor & found_inf, const at::Tensor & inv_scale);
  static ::std::tuple<::std::vector<at::Tensor>,at::Tensor> redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList self, const at::Tensor & found_inf, const at::Tensor & inv_scale);
};

}} // namespace at::_ops
