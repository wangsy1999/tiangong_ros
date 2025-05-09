#pragma once

// @generated by torchgen/gen.py from NativeFunction.h

#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>
#include <c10/core/QScheme.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <tuple>
#include <vector>


namespace at {
namespace native {

TORCH_API at::Tensor resize_functional(const at::Tensor & self, at::IntArrayRef size, c10::optional<at::MemoryFormat> memory_format=c10::nullopt);
TORCH_API const at::Tensor & resize_(const at::Tensor & self, at::IntArrayRef size, c10::optional<at::MemoryFormat> memory_format=c10::nullopt);
TORCH_API const at::Tensor & resize_cuda_(const at::Tensor & self, at::IntArrayRef size, c10::optional<at::MemoryFormat> memory_format=c10::nullopt);
TORCH_API const at::Tensor & resize_sparse_csr_(const at::Tensor & self, at::IntArrayRef size, c10::optional<at::MemoryFormat> memory_format=c10::nullopt);
TORCH_API const at::Tensor & quantized_resize_cpu_(const at::Tensor & self, at::IntArrayRef size, c10::optional<at::MemoryFormat> memory_format=c10::nullopt);

} // namespace native
} // namespace at
