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



#include <ATen/ops/can_cast_ops.h>

namespace at {


// aten::can_cast(ScalarType from, ScalarType to) -> bool
TORCH_API inline bool can_cast(at::ScalarType from, at::ScalarType to) {
    return at::_ops::can_cast::call(from, to);
}

}
