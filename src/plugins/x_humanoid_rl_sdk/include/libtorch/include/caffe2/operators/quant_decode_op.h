#ifndef QUANT_DECODE_OP_H_
#define QUANT_DECODE_OP_H_


#include <c10/util/irange.h>
#include <c10/util/typeid.h>
#include "caffe2/core/context.h"
#include "caffe2/core/operator.h"
#include "caffe2/core/tensor.h"

namespace caffe2 {

namespace {

template <class CodebookT, class CodeT>
void Decode(
    const Tensor& codebook,
    const Tensor& codes,
    /* optional */ const Tensor* const decoded_grad,
    Tensor* const output,
    bool resizeOnly) {
  CAFFE_ENFORCE(codebook.IsType<CodebookT>());

  auto* cb_ptr = codebook.data<CodebookT>();
  int cb_size = codebook.numel();

  CAFFE_ENFORCE(codes.IsType<CodeT>());
  auto* code_ptr = codes.data<CodeT>();

  if (decoded_grad == nullptr) {
    // Forward pass: decode and store codebook values in output.
    output->ResizeLike(codes);
    auto* out_ptr = output->template mutable_data<CodebookT>();
    if (resizeOnly) {
      return;
    }

    int sz = output->numel();
    for (C10_UNUSED const auto i : c10::irange(sz)) {
      DCHECK_LE(*code_ptr, cb_size);
      *out_ptr++ = cb_ptr[*code_ptr++];
    }
  } else {
    // Backward pass: decode and accumulate gradient w.r.t. codebook values.
    CAFFE_ENFORCE_EQ(codes.numel(), decoded_grad->numel());
    auto* gradient_ptr = decoded_grad->data<CodebookT>();
    auto* const gradient_end = gradient_ptr + decoded_grad->numel();

    CAFFE_ENFORCE_EQ(cb_size, output->numel());
    auto* out_ptr = output->template mutable_data<CodebookT>();
    while (gradient_ptr < gradient_end) {
      DCHECK_LE(*code_ptr, cb_size);
      out_ptr[*code_ptr++] += *gradient_ptr++;
    }
  }
}

#define REGISTER_DECODER(codebookType, codesType)                      \
  {                                                                    \
    {TypeMeta::Id<codebookType>(), TypeMeta::Id<codesType>()},         \
        [](const Tensor& codebook_,                                    \
           const Tensor& codes_,                                       \
           const Tensor* gradient_,                                    \
           Tensor* outDecoded_,                                        \
           bool resizeOnly_) {                                         \
          Decode<codebookType, codesType>(                             \
              codebook_, codes_, gradient_, outDecoded_, resizeOnly_); \
        }                                                              \
  }

inline void DecodeGeneral(
    const Tensor& codebook,
    const Tensor& codes,
    const Tensor* gradient,
    Tensor* outDecoded,
    bool resizeOnly) {
  const static std::map<
      std::pair<TypeIdentifier, TypeIdentifier>,
      std::function<void(
          const Tensor& codebook,
          const Tensor& codes,
          const Tensor* gradient,
          Tensor* outDecoded,
          bool resizeOnly)>>
      gDecoderMapper = {REGISTER_DECODER(float, uint8_t),
                        REGISTER_DECODER(float, uint16_t),
                        REGISTER_DECODER(float, int32_t)};

  gDecoderMapper.at({codebook.dtype().id(), codes.dtype().id()})(
      codebook, codes, gradient, outDecoded, resizeOnly);
}

} // namespace

// Decode tensors based on given codebook,
// The codebook is generated by model_quantize.py

enum class QuantDecodeRunTy {
  RUN_ALWAYS,
  RUN_ONCE,
};

template <QuantDecodeRunTy QuantDecodeRun>
class QuantDecodeOp final : public Operator<CPUContext> {
 public:
  USE_OPERATOR_FUNCTIONS(CPUContext);
  template <class... Args>
  explicit QuantDecodeOp(Args&&... args)
      : Operator<CPUContext>(std::forward<Args>(args)...) {}

  ~QuantDecodeOp() {}

  bool RunOnDevice() override {
    CAFFE_ENFORCE_GT(InputSize(), 1);
    // first input is the codebook
    CAFFE_ENFORCE_EQ(InputSize(), OutputSize() + 1);

    const auto& codebook = Input(0);
    CAFFE_ENFORCE(codebook.template IsType<float>(), codebook.dtype().name());

    for (const auto i : c10::irange(OutputSize())) {
      auto& ci = Input(i + 1);
      auto* co = Output(i);

      DecodeGeneral(
          codebook,
          ci,
          nullptr,
          co,
          /*resizeOnly=*/QuantDecodeRun == QuantDecodeRunTy::RUN_ONCE &&
              hasRun_);
    }
    hasRun_ = true;
    return true;
  }

 private:
  bool hasRun_{false};
};

class QuantDecodeGradientOp final : public Operator<CPUContext> {
 public:
  USE_OPERATOR_FUNCTIONS(CPUContext);
  template <class... Args>
  explicit QuantDecodeGradientOp(Args&&... args)
      : Operator<CPUContext>(std::forward<Args>(args)...) {}
  ~QuantDecodeGradientOp() {}

  bool RunOnDevice() override {
    // Inputs: 1 codebook, n tensors of codes, and n corresponding gradients.
    CAFFE_ENFORCE(InputSize() >= 3 && InputSize() % 2 == 1);
    const int num_code_tensors = (InputSize() - 1) / 2;
    CAFFE_ENFORCE_EQ(OutputSize(), 1);

    const auto& codebook = Input(0);
    CAFFE_ENFORCE(codebook.template IsType<float>(), codebook.dtype().name());

    auto* gradient = Output(0, codebook.sizes(), at::dtype<float>());
    auto* gradient_ptr = gradient->template mutable_data<float>();
    std::fill(gradient_ptr, gradient_ptr + gradient->numel(), 0);

    for (const auto i : c10::irange(num_code_tensors)) {
      auto& codes_i = Input(i + 1);
      auto& output_gradient_i = Input(i + num_code_tensors + 1);
      DecodeGeneral(codebook, codes_i, &output_gradient_i, gradient, false);
    }
    return true;
  }
};

} // namespace caffe2
#endif // QUANT_DECODE_OP_H_
