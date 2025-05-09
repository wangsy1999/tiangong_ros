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



#include <ATen/ops/dist_ops.h>

namespace at {


// aten::dist(Tensor self, Tensor other, Scalar p=2) -> Tensor
TORCH_API inline at::Tensor dist(const at::Tensor & self, const at::Tensor & other, const at::Scalar & p=2) {
    return at::_ops::dist::call(self, other, p);
}

}
