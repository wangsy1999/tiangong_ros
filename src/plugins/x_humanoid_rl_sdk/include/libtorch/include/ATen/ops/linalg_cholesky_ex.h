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



#include <ATen/ops/linalg_cholesky_ex_ops.h>

namespace at {


// aten::linalg_cholesky_ex(Tensor self, *, bool upper=False, bool check_errors=False) -> (Tensor L, Tensor info)
TORCH_API inline ::std::tuple<at::Tensor,at::Tensor> linalg_cholesky_ex(const at::Tensor & self, bool upper=false, bool check_errors=false) {
    return at::_ops::linalg_cholesky_ex::call(self, upper, check_errors);
}

// aten::linalg_cholesky_ex.L(Tensor self, *, bool upper=False, bool check_errors=False, Tensor(a!) L, Tensor(b!) info) -> (Tensor(a!) L, Tensor(b!) info)
TORCH_API inline ::std::tuple<at::Tensor &,at::Tensor &> linalg_cholesky_ex_out(at::Tensor & L, at::Tensor & info, const at::Tensor & self, bool upper=false, bool check_errors=false) {
    return at::_ops::linalg_cholesky_ex_L::call(self, upper, check_errors, L, info);
}

// aten::linalg_cholesky_ex.L(Tensor self, *, bool upper=False, bool check_errors=False, Tensor(a!) L, Tensor(b!) info) -> (Tensor(a!) L, Tensor(b!) info)
TORCH_API inline ::std::tuple<at::Tensor &,at::Tensor &> linalg_cholesky_ex_outf(const at::Tensor & self, bool upper, bool check_errors, at::Tensor & L, at::Tensor & info) {
    return at::_ops::linalg_cholesky_ex_L::call(self, upper, check_errors, L, info);
}

}
