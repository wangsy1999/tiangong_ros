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



#include <ATen/ops/reshape_ops.h>

namespace at {


// aten::reshape(Tensor(a) self, int[] shape) -> Tensor(a)
TORCH_API inline at::Tensor reshape(const at::Tensor & self, at::IntArrayRef shape) {
    return at::_ops::reshape::call(self, shape);
}

}
