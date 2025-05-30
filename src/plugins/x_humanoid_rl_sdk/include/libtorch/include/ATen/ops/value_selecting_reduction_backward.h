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



#include <ATen/ops/value_selecting_reduction_backward_ops.h>

namespace at {


// aten::value_selecting_reduction_backward(Tensor grad, int dim, Tensor indices, int[] sizes, bool keepdim) -> Tensor
TORCH_API inline at::Tensor value_selecting_reduction_backward(const at::Tensor & grad, int64_t dim, const at::Tensor & indices, at::IntArrayRef sizes, bool keepdim) {
    return at::_ops::value_selecting_reduction_backward::call(grad, dim, indices, sizes, keepdim);
}

}
