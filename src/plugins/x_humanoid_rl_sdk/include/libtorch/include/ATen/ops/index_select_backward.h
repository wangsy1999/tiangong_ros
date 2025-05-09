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



#include <ATen/ops/index_select_backward_ops.h>

namespace at {


// aten::index_select_backward(Tensor grad, int[] self_sizes, int dim, Tensor index) -> Tensor
TORCH_API inline at::Tensor index_select_backward(const at::Tensor & grad, at::IntArrayRef self_sizes, int64_t dim, const at::Tensor & index) {
    return at::_ops::index_select_backward::call(grad, self_sizes, dim, index);
}

}
