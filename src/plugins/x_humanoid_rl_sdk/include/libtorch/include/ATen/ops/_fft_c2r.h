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



#include <ATen/ops/_fft_c2r_ops.h>

namespace at {


// aten::_fft_c2r(Tensor self, int[] dim, int normalization, int last_dim_size) -> Tensor
TORCH_API inline at::Tensor _fft_c2r(const at::Tensor & self, at::IntArrayRef dim, int64_t normalization, int64_t last_dim_size) {
    return at::_ops::_fft_c2r::call(self, dim, normalization, last_dim_size);
}

// aten::_fft_c2r.out(Tensor self, int[] dim, int normalization, int last_dim_size, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & _fft_c2r_out(at::Tensor & out, const at::Tensor & self, at::IntArrayRef dim, int64_t normalization, int64_t last_dim_size) {
    return at::_ops::_fft_c2r_out::call(self, dim, normalization, last_dim_size, out);
}

// aten::_fft_c2r.out(Tensor self, int[] dim, int normalization, int last_dim_size, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & _fft_c2r_outf(const at::Tensor & self, at::IntArrayRef dim, int64_t normalization, int64_t last_dim_size, at::Tensor & out) {
    return at::_ops::_fft_c2r_out::call(self, dim, normalization, last_dim_size, out);
}

}
