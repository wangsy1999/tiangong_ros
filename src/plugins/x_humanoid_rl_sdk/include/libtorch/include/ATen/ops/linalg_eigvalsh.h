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



#include <ATen/ops/linalg_eigvalsh_ops.h>

namespace at {


// aten::linalg_eigvalsh(Tensor self, str UPLO="L") -> Tensor
TORCH_API inline at::Tensor linalg_eigvalsh(const at::Tensor & self, c10::string_view UPLO="L") {
    return at::_ops::linalg_eigvalsh::call(self, UPLO);
}

// aten::linalg_eigvalsh.out(Tensor self, str UPLO="L", *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & linalg_eigvalsh_out(at::Tensor & out, const at::Tensor & self, c10::string_view UPLO="L") {
    return at::_ops::linalg_eigvalsh_out::call(self, UPLO, out);
}

// aten::linalg_eigvalsh.out(Tensor self, str UPLO="L", *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & linalg_eigvalsh_outf(const at::Tensor & self, c10::string_view UPLO, at::Tensor & out) {
    return at::_ops::linalg_eigvalsh_out::call(self, UPLO, out);
}

}
