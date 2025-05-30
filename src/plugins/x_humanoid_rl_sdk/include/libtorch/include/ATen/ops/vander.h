#pragma once

// @generated by torchgen/gen.py from Function.h

#include <ATen/Context.h>
#include <ATen/DeviceGuard.h>
#include <ATen/TensorUtils.h>
#include <ATen/TracerMode.h>
#include <ATen/core/Generator.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>



#include <ATen/ops/vander_ops.h>

namespace at {


// aten::vander(Tensor x, int? N=None, bool increasing=False) -> Tensor
TORCH_API inline at::Tensor vander(const at::Tensor & x, c10::optional<int64_t> N=c10::nullopt, bool increasing=false) {
    return at::_ops::vander::call(x, N, increasing);
}

}
