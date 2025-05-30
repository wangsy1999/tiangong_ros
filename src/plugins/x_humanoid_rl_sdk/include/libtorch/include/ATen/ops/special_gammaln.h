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



#include <ATen/ops/special_gammaln_ops.h>

namespace at {


// aten::special_gammaln(Tensor self) -> Tensor
TORCH_API inline at::Tensor special_gammaln(const at::Tensor & self) {
    return at::_ops::special_gammaln::call(self);
}

// aten::special_gammaln.out(Tensor self, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_gammaln_out(at::Tensor & out, const at::Tensor & self) {
    return at::_ops::special_gammaln_out::call(self, out);
}

// aten::special_gammaln.out(Tensor self, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_gammaln_outf(const at::Tensor & self, at::Tensor & out) {
    return at::_ops::special_gammaln_out::call(self, out);
}

}
