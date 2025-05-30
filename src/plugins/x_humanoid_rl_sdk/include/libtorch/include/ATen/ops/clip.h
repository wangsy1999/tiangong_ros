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



#include <ATen/ops/clip_ops.h>

namespace at {


// aten::clip(Tensor self, Scalar? min=None, Scalar? max=None) -> Tensor
TORCH_API inline at::Tensor clip(const at::Tensor & self, const c10::optional<at::Scalar> & min, const c10::optional<at::Scalar> & max=c10::nullopt) {
    return at::_ops::clip::call(self, min, max);
}

// aten::clip.Tensor(Tensor self, Tensor? min=None, Tensor? max=None) -> Tensor
TORCH_API inline at::Tensor clip(const at::Tensor & self, const c10::optional<at::Tensor> & min={}, const c10::optional<at::Tensor> & max={}) {
    return at::_ops::clip_Tensor::call(self, min, max);
}

// aten::clip_(Tensor(a!) self, Scalar? min=None, Scalar? max=None) -> Tensor(a!)
TORCH_API inline at::Tensor & clip_(at::Tensor & self, const c10::optional<at::Scalar> & min, const c10::optional<at::Scalar> & max=c10::nullopt) {
    return at::_ops::clip_::call(self, min, max);
}

// aten::clip_.Tensor(Tensor(a!) self, Tensor? min=None, Tensor? max=None) -> Tensor(a!)
TORCH_API inline at::Tensor & clip_(at::Tensor & self, const c10::optional<at::Tensor> & min={}, const c10::optional<at::Tensor> & max={}) {
    return at::_ops::clip__Tensor::call(self, min, max);
}

// aten::clip.out(Tensor self, Scalar? min=None, Scalar? max=None, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & clip_out(at::Tensor & out, const at::Tensor & self, const c10::optional<at::Scalar> & min, const c10::optional<at::Scalar> & max=c10::nullopt) {
    return at::_ops::clip_out::call(self, min, max, out);
}

// aten::clip.out(Tensor self, Scalar? min=None, Scalar? max=None, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & clip_outf(const at::Tensor & self, const c10::optional<at::Scalar> & min, const c10::optional<at::Scalar> & max, at::Tensor & out) {
    return at::_ops::clip_out::call(self, min, max, out);
}

// aten::clip.Tensor_out(Tensor self, Tensor? min=None, Tensor? max=None, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & clip_out(at::Tensor & out, const at::Tensor & self, const c10::optional<at::Tensor> & min={}, const c10::optional<at::Tensor> & max={}) {
    return at::_ops::clip_Tensor_out::call(self, min, max, out);
}

// aten::clip.Tensor_out(Tensor self, Tensor? min=None, Tensor? max=None, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & clip_outf(const at::Tensor & self, const c10::optional<at::Tensor> & min, const c10::optional<at::Tensor> & max, at::Tensor & out) {
    return at::_ops::clip_Tensor_out::call(self, min, max, out);
}

}
