#pragma once

// @generated by torchgen/gen.py from NativeFunction.h

#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>
#include <c10/core/QScheme.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <tuple>
#include <vector>
#include <ATen/ops/clamp_meta.h>

namespace at {
namespace native {

TORCH_API at::Tensor & clamp_(at::Tensor & self, const c10::optional<at::Scalar> & min=c10::nullopt, const c10::optional<at::Scalar> & max=c10::nullopt);
struct TORCH_API structured_clamp_out : public at::meta::structured_clamp {
void impl(const at::Tensor & self, at::OptionalScalarRef min, at::OptionalScalarRef max, const at::Tensor & out);
};
TORCH_API at::Tensor clamp_quantized_cpu(const at::Tensor & self, const c10::optional<at::Scalar> & min=c10::nullopt, const c10::optional<at::Scalar> & max=c10::nullopt);
struct TORCH_API structured_clamp_Tensor_out : public at::meta::structured_clamp_Tensor {
void impl(const at::Tensor & self, at::OptionalTensorRef min, at::OptionalTensorRef max, const at::Tensor & out);
};

} // namespace native
} // namespace at
