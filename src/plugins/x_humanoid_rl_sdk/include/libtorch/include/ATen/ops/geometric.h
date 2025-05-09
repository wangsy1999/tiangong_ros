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



#include <ATen/ops/geometric_ops.h>

namespace at {


// aten::geometric.out(Tensor self, float p, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & geometric_out(at::Tensor & out, const at::Tensor & self, double p, c10::optional<at::Generator> generator=c10::nullopt) {
    return at::_ops::geometric_out::call(self, p, generator, out);
}

// aten::geometric.out(Tensor self, float p, *, Generator? generator=None, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & geometric_outf(const at::Tensor & self, double p, c10::optional<at::Generator> generator, at::Tensor & out) {
    return at::_ops::geometric_out::call(self, p, generator, out);
}

// aten::geometric.functional(Tensor self, float p, *, Generator? generator=None) -> Tensor
TORCH_API inline at::Tensor geometric_functional(const at::Tensor & self, double p, c10::optional<at::Generator> generator=c10::nullopt) {
    return at::_ops::geometric_functional::call(self, p, generator);
}

}
