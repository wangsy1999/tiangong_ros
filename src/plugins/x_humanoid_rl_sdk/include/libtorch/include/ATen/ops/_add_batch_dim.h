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



#include <ATen/ops/_add_batch_dim_ops.h>

namespace at {


// aten::_add_batch_dim(Tensor self, int batch_dim, int level) -> Tensor
TORCH_API inline at::Tensor _add_batch_dim(const at::Tensor & self, int64_t batch_dim, int64_t level) {
    return at::_ops::_add_batch_dim::call(self, batch_dim, level);
}

}
