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


struct TORCH_API quantize_per_tensor {
  using schema = at::Tensor (const at::Tensor &, double, int64_t, at::ScalarType);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::quantize_per_tensor")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "quantize_per_tensor(Tensor self, float scale, int zero_point, ScalarType dtype) -> Tensor")
  static at::Tensor call(const at::Tensor & self, double scale, int64_t zero_point, at::ScalarType dtype);
  static at::Tensor redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, double scale, int64_t zero_point, at::ScalarType dtype);
};

struct TORCH_API quantize_per_tensor_tensor_qparams {
  using schema = at::Tensor (const at::Tensor &, const at::Tensor &, const at::Tensor &, at::ScalarType);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::quantize_per_tensor")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "tensor_qparams")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "quantize_per_tensor.tensor_qparams(Tensor self, Tensor scale, Tensor zero_point, ScalarType dtype) -> Tensor")
  static at::Tensor call(const at::Tensor & self, const at::Tensor & scale, const at::Tensor & zero_point, at::ScalarType dtype);
  static at::Tensor redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & self, const at::Tensor & scale, const at::Tensor & zero_point, at::ScalarType dtype);
};

struct TORCH_API quantize_per_tensor_tensors {
  using schema = ::std::vector<at::Tensor> (at::TensorList, const at::Tensor &, const at::Tensor &, at::ScalarType);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::quantize_per_tensor")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "tensors")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "quantize_per_tensor.tensors(Tensor[] tensors, Tensor scales, Tensor zero_points, ScalarType dtype) -> Tensor[]")
  static ::std::vector<at::Tensor> call(at::TensorList tensors, const at::Tensor & scales, const at::Tensor & zero_points, at::ScalarType dtype);
  static ::std::vector<at::Tensor> redispatch(c10::DispatchKeySet dispatchKeySet, at::TensorList tensors, const at::Tensor & scales, const at::Tensor & zero_points, at::ScalarType dtype);
};

}} // namespace at::_ops
