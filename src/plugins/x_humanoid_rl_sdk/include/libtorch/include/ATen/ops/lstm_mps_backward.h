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



#include <ATen/ops/lstm_mps_backward_ops.h>

namespace at {


// aten::lstm_mps_backward(Tensor grad_y, Tensor? grad_hy, Tensor? grad_cy, Tensor z_state, Tensor cell_state_fwd, Tensor input, Tensor[] hx, Tensor[] params, bool has_biases, int num_layers, float dropout, bool train, bool bidirectional, bool batch_first) -> (Tensor, Tensor[], Tensor[])
TORCH_API inline ::std::tuple<at::Tensor,::std::vector<at::Tensor>,::std::vector<at::Tensor>> lstm_mps_backward(const at::Tensor & grad_y, const c10::optional<at::Tensor> & grad_hy, const c10::optional<at::Tensor> & grad_cy, const at::Tensor & z_state, const at::Tensor & cell_state_fwd, const at::Tensor & input, at::TensorList hx, at::TensorList params, bool has_biases, int64_t num_layers, double dropout, bool train, bool bidirectional, bool batch_first) {
    return at::_ops::lstm_mps_backward::call(grad_y, grad_hy, grad_cy, z_state, cell_state_fwd, input, hx, params, has_biases, num_layers, dropout, train, bidirectional, batch_first);
}

}
