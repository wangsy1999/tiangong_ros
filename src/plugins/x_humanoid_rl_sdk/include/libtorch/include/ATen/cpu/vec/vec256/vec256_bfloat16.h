#pragma once

// DO NOT DEFINE STATIC DATA IN THIS HEADER!
// See Note [Do not compile initializers with AVX]

#include <ATen/cpu/vec/intrinsics.h>
#include <ATen/cpu/vec/vec_base.h>
#include <c10/util/irange.h>

#if defined(CPU_CAPABILITY_AVX2) && !defined(_MSC_VER)
#include <sleef.h>
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"

namespace at {
namespace vec {
// See Note [CPU_CAPABILITY namespace]
inline namespace CPU_CAPABILITY {

#if defined(CPU_CAPABILITY_AVX2) && !defined(_MSC_VER)

static inline void cvtbf16_fp32(const __m128i& a, __m256& o) {
  o = _mm256_castsi256_ps(_mm256_slli_epi32(_mm256_cvtepu16_epi32(a), 16));
}

static inline void cvtbf16_fp32(const __m256i& a, __m256& o1, __m256& o2) {
  __m128i lo = _mm256_extractf128_si256(a, 0);
  __m128i hi = _mm256_extractf128_si256(a, 1);
  cvtbf16_fp32(lo, o1);
  cvtbf16_fp32(hi, o2);
}
static inline __m256i cvtfp32_bf16(const __m256& a, const __m256& b) {
  __m256i lo = _mm256_castps_si256(a);
  __m256i hi = _mm256_castps_si256(b);
  __m256i nan = _mm256_set1_epi32(0xffff);
  __m256i mask_lo = _mm256_castps_si256(_mm256_cmp_ps(a, a, _CMP_ORD_Q));
  __m256i mask_hi = _mm256_castps_si256(_mm256_cmp_ps(b, b, _CMP_ORD_Q));
  __m256i ones = _mm256_set1_epi32(0x1);
  __m256i vec_bias = _mm256_set1_epi32(0x7fff);
  // uint32_t lsb = (input >> 16) & 1;
  auto t_lo = _mm256_and_si256(_mm256_srli_epi32(lo, 16), ones);
  auto t_hi = _mm256_and_si256(_mm256_srli_epi32(hi, 16), ones);
  // uint32_t rounding_bias = 0x7fff + lsb;
  t_lo = _mm256_add_epi32(t_lo, vec_bias);
  t_hi = _mm256_add_epi32(t_hi, vec_bias);
  // input += rounding_bias;
  t_lo = _mm256_add_epi32(t_lo, lo);
  t_hi = _mm256_add_epi32(t_hi, hi);
  // input = input >> 16;
  t_lo = _mm256_srli_epi32(t_lo, 16);
  t_hi = _mm256_srli_epi32(t_hi, 16);
  // Check NaN before converting back to bf16
  t_lo = _mm256_blendv_epi8(nan, t_lo, mask_lo);
  t_hi = _mm256_blendv_epi8(nan, t_hi, mask_hi);

  t_lo = _mm256_packus_epi32(t_lo, t_hi);      // t_hi[4-7] t_lo[4-7] t_hi[0-4] t_lo[0-4]
  return _mm256_permute4x64_epi64(t_lo, 0xd8); // 11        01        10        00
}

static inline __m256i merge_compare_result(const __m256& a, const __m256& b) {
  __m256i lo = _mm256_castps_si256(a);
  __m256i hi = _mm256_castps_si256(b);
  lo = _mm256_srli_epi32(lo, 16);
  hi = _mm256_srli_epi32(hi, 16);
  auto out = _mm256_packus_epi32(lo, hi);
  return _mm256_permute4x64_epi64(out, 0xd8);
}

template <> class Vectorized<BFloat16> {
private:
  __m256i values;
public:
  using value_type = uint16_t;
  using size_type = int;
  static constexpr size_type size() {
    return 16;
  }
  Vectorized() {}
  Vectorized(__m256i v) : values(v) {}
  Vectorized(BFloat16 val) {
    value_type uw = val.x;
    values = _mm256_set1_epi16(uw);
  }
  Vectorized(BFloat16 val1, BFloat16 val2, BFloat16 val3, BFloat16 val4,
         BFloat16 val5, BFloat16 val6, BFloat16 val7, BFloat16 val8,
         BFloat16 val9, BFloat16 val10, BFloat16 val11, BFloat16 val12,
         BFloat16 val13, BFloat16 val14, BFloat16 val15, BFloat16 val16) {
    values = _mm256_setr_epi16(
        val1.x, val2.x, val3.x, val4.x, val5.x, val6.x, val7.x, val8.x,
        val9.x, val10.x, val11.x, val12.x, val13.x, val14.x, val15.x, val16.x);
  }
  operator __m256i() const {
    return values;
  }
  BFloat16& operator[](int idx) = delete;
  const BFloat16& operator[](int idx) const  = delete;
  int zero_mask() const {
    // returns an integer mask where all zero elements are translated to 1-bit and others are translated to 0-bit
    __m256i cmp = _mm256_cmpeq_epi16(values, _mm256_set1_epi16(0));
    return _mm256_movemask_epi8(cmp);
  }
  static Vectorized<BFloat16> loadu(const void* ptr) {
    return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(ptr));
  }
  static Vectorized<BFloat16> loadu(const void* ptr, int16_t count) {
    __at_align__ int16_t tmp_values[size()];
    std::memcpy(tmp_values, ptr, count * sizeof(int16_t));
    return loadu(tmp_values);
  }
  void store(void* ptr, int count = size()) const {
    if (count == size()) {
      _mm256_storeu_si256(reinterpret_cast<__m256i*>(ptr), values);
    } else if (count > 0) {
      __at_align__ int16_t tmp_values[size()];
      _mm256_storeu_si256(reinterpret_cast<__m256i*>(tmp_values), values);
      std::memcpy(ptr, tmp_values, count * sizeof(int16_t));
    }
  }
  template <int64_t mask>
  static Vectorized<BFloat16> blend(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
    __at_align__ int16_t tmp_values[size()];
    a.store(tmp_values);
    if (mask & 0x01)
      tmp_values[0] = _mm256_extract_epi16(b.values, 0);
    if (mask & 0x02)
      tmp_values[1] = _mm256_extract_epi16(b.values, 1);
    if (mask & 0x04)
      tmp_values[2] = _mm256_extract_epi16(b.values, 2);
    if (mask & 0x08)
      tmp_values[3] = _mm256_extract_epi16(b.values, 3);
    if (mask & 0x10)
      tmp_values[4] = _mm256_extract_epi16(b.values, 4);
    if (mask & 0x20)
      tmp_values[5] = _mm256_extract_epi16(b.values, 5);
    if (mask & 0x40)
      tmp_values[6] = _mm256_extract_epi16(b.values, 6);
    if (mask & 0x80)
      tmp_values[7] = _mm256_extract_epi16(b.values, 7);
    if (mask & 0x100)
      tmp_values[8] = _mm256_extract_epi16(b.values, 8);
    if (mask & 0x200)
      tmp_values[9] = _mm256_extract_epi16(b.values, 9);
    if (mask & 0x400)
      tmp_values[10] = _mm256_extract_epi16(b.values, 10);
    if (mask & 0x800)
      tmp_values[11] = _mm256_extract_epi16(b.values, 11);
    if (mask & 0x1000)
      tmp_values[12] = _mm256_extract_epi16(b.values, 12);
    if (mask & 0x2000)
      tmp_values[13] = _mm256_extract_epi16(b.values, 13);
    if (mask & 0x4000)
      tmp_values[14] = _mm256_extract_epi16(b.values, 14);
    if (mask & 0x8000)
      tmp_values[15] = _mm256_extract_epi16(b.values, 15);
    return loadu(tmp_values);
  }
  static Vectorized<BFloat16> blendv(const Vectorized<BFloat16>& a,
      const Vectorized<BFloat16>& b, const Vectorized<BFloat16>& mask) {
    return _mm256_blendv_epi8(a.values, b.values, mask.values);
  }
  template<typename step_t>
  static Vectorized<BFloat16> arange(BFloat16 base = 0.f, step_t step = static_cast<step_t>(1)) {
    return Vectorized<BFloat16>(
      base,             base +      step, base +  2 * step, base +  3 * step,
      base +  4 * step, base +  5 * step, base +  6 * step, base +  7 * step,
      base +  8 * step, base +  9 * step, base + 10 * step, base + 11 * step,
      base + 12 * step, base + 13 * step, base + 14 * step, base + 15 * step);
  }
  static Vectorized<BFloat16> set(const Vectorized<BFloat16>& a,
      const Vectorized<BFloat16>& b, int64_t count = size()) {
    switch (count) {
      case 0:
        return a;
      case 1:
        return blend<1>(a, b);
      case 2:
        return blend<3>(a, b);
      case 3:
        return blend<7>(a, b);
      case 4:
        return blend<15>(a, b);
      case 5:
        return blend<31>(a, b);
      case 6:
        return blend<63>(a, b);
      case 7:
        return blend<127>(a, b);
      case 8:
        return blend<255>(a, b);
      case 9:
        return blend<511>(a, b);
      case 10:
        return blend<1023>(a, b);
      case 11:
        return blend<2047>(a, b);
      case 12:
        return blend<4095>(a, b);
      case 13:
        return blend<8191>(a, b);
      case 14:
        return blend<16383>(a, b);
      case 15:
        return blend<32767>(a, b);
    }
    return b;
  }
  Vectorized<BFloat16> map(const __m256 (*const vop)(__m256)) const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    const auto o1 = vop(lo);
    const auto o2 = vop(hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> abs() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    const auto mask = _mm256_set1_ps(-0.f);
    const auto o1 = _mm256_andnot_ps(mask, lo);
    const auto o2 = _mm256_andnot_ps(mask, hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> angle() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto angle_lambda = [](__m256 values) {
      const auto zero_vec = _mm256_set1_ps(0.f);
      const auto nan_vec = _mm256_set1_ps(NAN);
      const auto not_nan_mask = _mm256_cmp_ps(values, values, _CMP_EQ_OQ);
      const auto nan_mask = _mm256_cmp_ps(not_nan_mask, zero_vec, _CMP_EQ_OQ);
      const auto pi = _mm256_set1_ps(c10::pi<float>);

      const auto neg_mask = _mm256_cmp_ps(values, zero_vec, _CMP_LT_OQ);
      auto angle = _mm256_blendv_ps(zero_vec, pi, neg_mask);
      angle = _mm256_blendv_ps(angle, nan_vec, nan_mask);
      return angle;
    };
    auto o1 = angle_lambda(lo);
    auto o2 = angle_lambda(hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> real() const {
    return *this;
  }
  Vectorized<BFloat16> imag() const {
    return _mm256_set1_epi16(0);
  }
  Vectorized<BFloat16> conj() const {
    return *this;
  }
  Vectorized<BFloat16> acos() const {
    return map(Sleef_acosf8_u10);
  }
  Vectorized<BFloat16> asin() const {
    return map(Sleef_asinf8_u10);
  }
  Vectorized<BFloat16> atan() const {
    return map(Sleef_atanf8_u10);
  }
  Vectorized<BFloat16> atan2(const Vectorized<BFloat16> &b) const {
    __m256 lo, hi;
    __m256 b1, b2;
    cvtbf16_fp32(values, lo, hi);
    cvtbf16_fp32(b.values, b1, b2);
    auto o1 = Sleef_atan2f8_u10(lo, b1);
    auto o2 = Sleef_atan2f8_u10(hi, b2);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> copysign(const Vectorized<BFloat16> &sign) const {
    // copy sign bit (0x8000) from sign and remaining bits from values
    __m256i mask_value = _mm256_set1_epi32(~0x80008000);
    __m256i mask_signbit = _mm256_set1_epi32(0x80008000);
    return Vectorized<BFloat16>(
      _mm256_or_si256(
        _mm256_and_si256(values, mask_value),
        _mm256_and_si256(sign, mask_signbit)));
  }
  Vectorized<BFloat16> erf() const {
    return map(Sleef_erff8_u10);
  }
  Vectorized<BFloat16> erfc() const {
    return map(Sleef_erfcf8_u15);
  }
  Vectorized<BFloat16> erfinv() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    __at_align__ float tmp1[size() / 2], tmp2[size() / 2];
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp1), lo);
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp2), hi);
    for (int64_t i = 0; i < size() / 2; i++) {
      tmp1[i] = calc_erfinv(tmp1[i]);
      tmp2[i] = calc_erfinv(tmp2[i]);
    }
    auto o1 = _mm256_loadu_ps(tmp1);
    auto o2 = _mm256_loadu_ps(tmp2);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> exp() const {
    return map(Sleef_expf8_u10);
  }
  Vectorized<BFloat16> expm1() const {
    return map(Sleef_expm1f8_u10);
  }
  Vectorized<BFloat16> fmod(const Vectorized<BFloat16> & q) const {
    __m256 x_lo, x_hi;
    cvtbf16_fp32(values, x_lo, x_hi);
    __m256 q_lo, q_hi;
    cvtbf16_fp32(q.values, q_lo, q_hi);
    auto o1 = Sleef_fmodf8(x_lo, q_lo);
    auto o2 = Sleef_fmodf8(x_hi, q_hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> hypot(const Vectorized<BFloat16> &b) const {
    __m256 lo, hi;
    __m256 b1, b2;
    cvtbf16_fp32(values, lo, hi);
    cvtbf16_fp32(b.values, b1, b2);
    auto o1 = Sleef_hypotf8_u05(lo, b1);
    auto o2 = Sleef_hypotf8_u05(hi, b2);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> i0() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    __at_align__ float tmp1[size() / 2], tmp2[size() / 2];
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp1), lo);
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp2), hi);
    for (int64_t i = 0; i < size() / 2; i++) {
      tmp1[i] = calc_i0(tmp1[i]);
      tmp2[i] = calc_i0(tmp2[i]);
    }
    auto o1 = _mm256_loadu_ps(tmp1);
    auto o2 = _mm256_loadu_ps(tmp2);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> i0e() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    constexpr auto sz = size();
    __at_align__ float tmp1[sz / 2], tmp2[sz / 2];
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp1), lo);
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp2), hi);

    for (auto i = decltype(sz){0}; i < sz / 2; i++) {
      tmp1[i] = calc_i0e(tmp1[i]);
      tmp2[i] = calc_i0e(tmp2[i]);
    }
    const auto o1 = _mm256_loadu_ps(tmp1);
    const auto o2 = _mm256_loadu_ps(tmp2);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> igamma(const Vectorized<BFloat16> &x) const {
    __m256 lo, hi;
    __m256 xlo, xhi;
    cvtbf16_fp32(values, lo, hi);
    cvtbf16_fp32(x.values, xlo, xhi);
    __at_align__ float tmp1[size() / 2], tmp2[size() / 2];
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp1), lo);
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp2), hi);
    __at_align__ float tmpx1[size() / 2], tmpx2[size() / 2];
    _mm256_storeu_ps(reinterpret_cast<float*>(tmpx1), xlo);
    _mm256_storeu_ps(reinterpret_cast<float*>(tmpx2), xhi);
    for (int64_t i = 0; i < size() / 2; ++i) {
      tmp1[i] = calc_igamma(tmp1[i], tmpx1[i]);
      tmp2[i] = calc_igamma(tmp2[i], tmpx2[i]);
    }
    auto o1 = _mm256_loadu_ps(tmp1);
    auto o2 = _mm256_loadu_ps(tmp2);
    return cvtfp32_bf16(o1, o2);
  }

  Vectorized<BFloat16> igammac(const Vectorized<BFloat16> &x) const {
    __m256 lo, hi;
    __m256 xlo, xhi;
    cvtbf16_fp32(values, lo, hi);
    cvtbf16_fp32(x.values, xlo, xhi);
    __at_align__ float tmp1[size() / 2], tmp2[size() / 2];
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp1), lo);
    _mm256_storeu_ps(reinterpret_cast<float*>(tmp2), hi);
    __at_align__ float tmpx1[size() / 2], tmpx2[size() / 2];
    _mm256_storeu_ps(reinterpret_cast<float*>(tmpx1), xlo);
    _mm256_storeu_ps(reinterpret_cast<float*>(tmpx2), xhi);
    for (int64_t i = 0; i < size() / 2; ++i) {
      tmp1[i] = calc_igammac(tmp1[i], tmpx1[i]);
      tmp2[i] = calc_igammac(tmp2[i], tmpx2[i]);
    }
    auto o1 = _mm256_loadu_ps(tmp1);
    auto o2 = _mm256_loadu_ps(tmp2);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> log() const {
    return map(Sleef_logf8_u10);
  }
  Vectorized<BFloat16> log2() const {
    return map(Sleef_log2f8_u10);
  }
  Vectorized<BFloat16> log10() const {
    return map(Sleef_log10f8_u10);
  }
  Vectorized<BFloat16> log1p() const {
    return map(Sleef_log1pf8_u10);
  }
  Vectorized<BFloat16> frac() const;
  Vectorized<BFloat16> sin() const {
    return map(Sleef_sinf8_u10);
  }
  Vectorized<BFloat16> sinh() const {
    return map(Sleef_sinhf8_u10);
  }
  Vectorized<BFloat16> cos() const {
    return map(Sleef_cosf8_u10);
  }
  Vectorized<BFloat16> cosh() const {
    return map(Sleef_coshf8_u10);
  }
  Vectorized<BFloat16> ceil() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto o1 = _mm256_ceil_ps(lo);
    auto o2 = _mm256_ceil_ps(hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> floor() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto o1 = _mm256_floor_ps(lo);
    auto o2 = _mm256_floor_ps(hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> neg() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto mask = _mm256_set1_ps(-0.f);
    auto o1 = _mm256_xor_ps(mask, lo);
    auto o2 = _mm256_xor_ps(mask, hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> round() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto o1 = _mm256_round_ps(lo, (_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
    auto o2 = _mm256_round_ps(hi, (_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> tan() const {
    return map(Sleef_tanf8_u10);
  }
  Vectorized<BFloat16> tanh() const {
    return map(Sleef_tanhf8_u10);
  }
  Vectorized<BFloat16> trunc() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto o1 = _mm256_round_ps(lo, (_MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
    auto o2 = _mm256_round_ps(hi, (_MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> lgamma() const {
    return map(Sleef_lgammaf8_u10);
  }
  Vectorized<BFloat16> sqrt() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto o1 = _mm256_sqrt_ps(lo);
    auto o2 = _mm256_sqrt_ps(hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> reciprocal() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto ones = _mm256_set1_ps(1);
    auto o1 = _mm256_div_ps(ones, lo);
    auto o2 = _mm256_div_ps(ones, hi);
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> rsqrt() const {
    __m256 lo, hi;
    cvtbf16_fp32(values, lo, hi);
    auto ones = _mm256_set1_ps(1);
    auto o1 = _mm256_div_ps(ones, _mm256_sqrt_ps(lo));
    auto o2 = _mm256_div_ps(ones, _mm256_sqrt_ps(hi));
    return cvtfp32_bf16(o1, o2);
  }
  Vectorized<BFloat16> pow(const Vectorized<BFloat16> &b) const {
    __m256 lo, hi;
    __m256 b1, b2;
    cvtbf16_fp32(values, lo, hi);
    cvtbf16_fp32(b.values, b1, b2);
    auto o1 = Sleef_powf8_u10(lo, b1);
    auto o2 = Sleef_powf8_u10(hi, b2);
    return cvtfp32_bf16(o1, o2);
  }

  Vectorized<BFloat16> inline operator>(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> inline operator<(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> inline operator>=(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> inline operator<=(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> inline operator==(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> inline operator!=(const Vectorized<BFloat16>& other) const;

  Vectorized<BFloat16> eq(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> ne(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> gt(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> ge(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> lt(const Vectorized<BFloat16>& other) const;
  Vectorized<BFloat16> le(const Vectorized<BFloat16>& other) const;
};

template<typename Op>
Vectorized<BFloat16> static inline bfloat16_binary_op_as_fp32(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b, Op op) {
  __m256 a_lo, a_hi;
  __m256 b_lo, b_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(b), b_lo, b_hi);
  auto o1 = op(a_lo, b_lo);
  auto o2 = op(a_hi, b_hi);
  return cvtfp32_bf16(o1, o2);
}

template<typename Op>
Vectorized<BFloat16> static inline bfloat16_compare_as_fp32(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b, Op op) {
  __m256 a_lo, a_hi;
  __m256 b_lo, b_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(b), b_lo, b_hi);
  auto o1 = op(a_lo, b_lo);
  auto o2 = op(a_hi, b_hi);
  return merge_compare_result(o1, o2);
}

Vectorized<BFloat16> inline Vectorized<BFloat16>::operator>(const Vectorized<BFloat16>& other) const {
  return bfloat16_compare_as_fp32(*this, other, [](__m256 x, __m256 y) {
    return _mm256_cmp_ps(x, y, _CMP_GT_OQ);
  });
}
Vectorized<BFloat16> inline Vectorized<BFloat16>::operator<(const Vectorized<BFloat16>& other) const {
  return bfloat16_compare_as_fp32(*this, other, [](__m256 x, __m256 y) {
    return _mm256_cmp_ps(x, y, _CMP_LT_OQ);
  });
}
Vectorized<BFloat16> inline Vectorized<BFloat16>::operator>=(const Vectorized<BFloat16>& other) const {
  return bfloat16_compare_as_fp32(*this, other, [](__m256 x, __m256 y) {
    return _mm256_cmp_ps(x, y, _CMP_GE_OQ);
  });
}
Vectorized<BFloat16> inline Vectorized<BFloat16>::operator<=(const Vectorized<BFloat16>& other) const {
  return bfloat16_compare_as_fp32(*this, other, [](__m256 x, __m256 y) {
    return _mm256_cmp_ps(x, y, _CMP_LE_OQ);
  });
}
Vectorized<BFloat16> inline Vectorized<BFloat16>::operator==(const Vectorized<BFloat16>& other) const {
  return bfloat16_compare_as_fp32(*this, other, [](__m256 x, __m256 y) {
    return _mm256_cmp_ps(x, y, _CMP_EQ_OQ);
  });
}
Vectorized<BFloat16> inline Vectorized<BFloat16>::operator!=(const Vectorized<BFloat16>& other) const {
  return bfloat16_compare_as_fp32(*this, other, [](__m256 x, __m256 y) {
    return _mm256_cmp_ps(x, y, _CMP_NEQ_UQ);
  });
}

Vectorized<BFloat16> inline operator+(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  return bfloat16_binary_op_as_fp32(a, b, [](const __m256& x, const __m256& y) { return _mm256_add_ps(x, y); });
}
Vectorized<BFloat16> inline operator-(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  return bfloat16_binary_op_as_fp32(a, b, [](const __m256& x, const __m256& y) { return _mm256_sub_ps(x, y); });
}
Vectorized<BFloat16> inline operator*(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  return bfloat16_binary_op_as_fp32(a, b, [](const __m256& x, const __m256& y) { return _mm256_mul_ps(x, y); });
}
Vectorized<BFloat16> inline operator/(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  return bfloat16_binary_op_as_fp32(a, b, [](const __m256& x, const __m256& y) { return _mm256_div_ps(x, y); });
}

Vectorized<BFloat16> inline operator&(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  return _mm256_and_si256(a, b);
}
Vectorized<BFloat16> inline operator|(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  return _mm256_or_si256(a, b);
}
Vectorized<BFloat16> inline operator^(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  return _mm256_xor_si256(a, b);
}

inline Vectorized<BFloat16> Vectorized<BFloat16>::eq(const Vectorized<BFloat16>& other) const {
  return (*this == other) & Vectorized<BFloat16>(1.0f);
}

inline Vectorized<BFloat16> Vectorized<BFloat16>::ne(const Vectorized<BFloat16>& other) const {
  return (*this != other) & Vectorized<BFloat16>(1.0f);
}

inline Vectorized<BFloat16> Vectorized<BFloat16>::gt(const Vectorized<BFloat16>& other) const {
  return (*this > other) & Vectorized<BFloat16>(1.0f);
}

inline Vectorized<BFloat16> Vectorized<BFloat16>::ge(const Vectorized<BFloat16>& other) const {
  return (*this >= other) & Vectorized<BFloat16>(1.0f);
}

inline Vectorized<BFloat16> Vectorized<BFloat16>::lt(const Vectorized<BFloat16>& other) const {
  return (*this < other) & Vectorized<BFloat16>(1.0f);
}

inline Vectorized<BFloat16> Vectorized<BFloat16>::le(const Vectorized<BFloat16>& other) const {
  return (*this <= other) & Vectorized<BFloat16>(1.0f);
}

// frac. Implement this here so we can use subtraction
inline Vectorized<BFloat16> Vectorized<BFloat16>::frac() const {
  return *this - this->trunc();
}

// Implements the IEEE 754 201X `maximum` operation, which propagates NaN if
// either input is a NaN.
template <>
Vectorized<BFloat16> inline maximum(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  __m256 a_lo, a_hi;
  __m256 b_lo, b_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(b), b_lo, b_hi);
  auto max_lo = _mm256_max_ps(a_lo, b_lo);
  auto max_hi = _mm256_max_ps(a_hi, b_hi);
  auto nan_lo = _mm256_cmp_ps(a_lo, b_lo, _CMP_UNORD_Q);
  auto nan_hi = _mm256_cmp_ps(a_hi, b_hi, _CMP_UNORD_Q);
  // Exploit the fact that all-ones is a NaN.
  auto o1 = _mm256_or_ps(max_lo, nan_lo);
  auto o2 = _mm256_or_ps(max_hi, nan_hi);
  return cvtfp32_bf16(o1, o2);
}

// Implements the IEEE 754 201X `minimum` operation, which propagates NaN if
// either input is a NaN.
template <>
Vectorized<BFloat16> inline minimum(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& b) {
  __m256 a_lo, a_hi;
  __m256 b_lo, b_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(b), b_lo, b_hi);
  auto min_lo = _mm256_min_ps(a_lo, b_lo);
  auto min_hi = _mm256_min_ps(a_hi, b_hi);
  auto nan_lo = _mm256_cmp_ps(a_lo, b_lo, _CMP_UNORD_Q);
  auto nan_hi = _mm256_cmp_ps(a_hi, b_hi, _CMP_UNORD_Q);
  // Exploit the fact that all-ones is a NaN.
  auto o1 = _mm256_or_ps(min_lo, nan_lo);
  auto o2 = _mm256_or_ps(min_hi, nan_hi);
  return cvtfp32_bf16(o1, o2);
}

template <>
Vectorized<BFloat16> inline clamp(const Vectorized<BFloat16>& a,
    const Vectorized<BFloat16>& min, const Vectorized<BFloat16>& max) {
  __m256 a_lo, a_hi;
  __m256 min_lo, min_hi;
  __m256 max_lo, max_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(min), min_lo, min_hi);
  cvtbf16_fp32(__m256i(max), max_lo, max_hi);
  auto o1 = _mm256_min_ps(max_lo, _mm256_max_ps(min_lo, a_lo));
  auto o2 = _mm256_min_ps(max_hi, _mm256_max_ps(min_hi, a_hi));
  return cvtfp32_bf16(o1, o2);
}

template <>
Vectorized<BFloat16> inline clamp_max(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& max) {
  __m256 a_lo, a_hi;
  __m256 max_lo, max_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(max), max_lo, max_hi);
  auto o1 = _mm256_min_ps(max_lo, a_lo);
  auto o2 = _mm256_min_ps(max_hi, a_hi);
  return cvtfp32_bf16(o1, o2);
}

template <>
Vectorized<BFloat16> inline clamp_min(const Vectorized<BFloat16>& a, const Vectorized<BFloat16>& min) {
  __m256 a_lo, a_hi;
  __m256 min_lo, min_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(min), min_lo, min_hi);
  auto o1 = _mm256_max_ps(min_lo, a_lo);
  auto o2 = _mm256_max_ps(min_hi, a_hi);
  return cvtfp32_bf16(o1, o2);
}

template <>
inline void convert(const BFloat16* src, BFloat16* dst, int64_t n) {
  int64_t i;
#pragma unroll
  for (i = 0; i <= (n - Vectorized<BFloat16>::size()); i += Vectorized<BFloat16>::size()) {
    auto vsrc = _mm256_loadu_si256(reinterpret_cast<__m256i*>((void*)(src + i)));
    _mm256_storeu_si256(reinterpret_cast<__m256i*>((void*)(dst + i)), vsrc);
  }
#pragma unroll
  for (; i < n; i++) {
    dst[i] = src[i];
  }
}

template <>
inline void convert(const BFloat16* src, float* dst, int64_t n) {
  int64_t i;
#pragma unroll
  for (i = 0; i <= (n - Vectorized<BFloat16>::size()); i += Vectorized<BFloat16>::size()) {
    auto vsrc = _mm256_loadu_si256(reinterpret_cast<__m256i*>((void*)(src + i)));
    __m256 o1, o2;
    cvtbf16_fp32(vsrc, o1, o2);
    _mm256_storeu_ps(dst + i, o1);
    _mm256_storeu_ps(dst + i + Vectorized<float>::size(), o2);
  }
#pragma unroll
  for (; i < n; i++) {
    dst[i] = static_cast<float>(src[i]);
  }
}

template <>
Vectorized<BFloat16> inline fmadd(const Vectorized<BFloat16>& a,
    const Vectorized<BFloat16>& b, const Vectorized<BFloat16>& c) {
  __m256 a_lo, a_hi;
  __m256 b_lo, b_hi;
  __m256 c_lo, c_hi;
  cvtbf16_fp32(__m256i(a), a_lo, a_hi);
  cvtbf16_fp32(__m256i(b), b_lo, b_hi);
  cvtbf16_fp32(__m256i(c), c_lo, c_hi);
  auto o1 = _mm256_fmadd_ps(a_lo, b_lo, c_lo);
  auto o2 = _mm256_fmadd_ps(a_hi, b_hi, c_hi);
  return cvtfp32_bf16(o1, o2);
}

inline std::tuple<Vectorized<float>, Vectorized<float>> convert_bfloat16_float(const Vectorized<BFloat16>& a) {
  __m256 o1, o2;
  cvtbf16_fp32(__m256i(a), o1, o2);
  return std::make_tuple(o1, o2);
}

inline Vectorized<BFloat16> convert_float_bfloat16(const Vectorized<float>& a, const Vectorized<float>& b) {
 return cvtfp32_bf16(__m256(a), __m256(b));
}


#else // defined(CPU_CAPABILITY_AVX2) && !defined(_MSC_VER)

inline std::tuple<Vectorized<float>, Vectorized<float>> convert_bfloat16_float(const Vectorized<BFloat16>& a) {
  constexpr int64_t K = Vectorized<BFloat16>::size();
  __at_align__ float arr[K];
  __at_align__ BFloat16 arr2[K];
  a.store(arr2);
  convert(arr2, arr, K);
  return std::make_tuple(
      Vectorized<float>::loadu(arr),
      Vectorized<float>::loadu(arr + Vectorized<float>::size()));
}

inline Vectorized<BFloat16> convert_float_bfloat16(const Vectorized<float>& a, const Vectorized<float>& b) {
  constexpr int64_t K = Vectorized<BFloat16>::size();
  __at_align__ float arr[K];
  __at_align__ BFloat16 arr2[K];
  a.store(arr);
  b.store(arr + Vectorized<float>::size());
  convert(arr, arr2, K);
  return Vectorized<BFloat16>::loadu(arr2);
}

#endif // defined(CPU_CAPABILITY_AVX2) && !defined(_MSC_VER)

#if defined(CPU_CAPABILITY_AVX2) && !defined(_MSC_VER)
inline void load_fp32_from_bf16(const c10::BFloat16 *data, Vectorized<float>& out) {
  auto values = _mm_loadu_si128(reinterpret_cast<const __m128i*>(data));
  __m256 out_values;
  cvtbf16_fp32(values, out_values);
  out = out_values;
}

inline void load_fp32_from_bf16(const c10::BFloat16 *data, Vectorized<float>& out1, Vectorized<float>& out2) {
  auto vec = Vectorized<c10::BFloat16>::loadu(data);
  __m256 out1_values, out2_values;
  cvtbf16_fp32(vec, out1_values, out2_values);
  out1 = out1_values;
  out2 = out2_values;
}
#else // defined(CPU_CAPABILITY_AVX2) && !defined(_MSC_VER)
inline void load_fp32_from_bf16(const c10::BFloat16 *data, Vectorized<float>& out) {
  __at_align__ float values[Vectorized<float>::size()];
  for (const auto k : c10::irange(Vectorized<float>::size())) {
    values[k] = data[k];
  }
  out = Vectorized<float>::loadu(values);
}

inline void load_fp32_from_bf16(const c10::BFloat16 *data, Vectorized<float>& out1, Vectorized<float>& out2) {
  load_fp32_from_bf16(data, out1);
  data += Vectorized<float>::size();
  load_fp32_from_bf16(data, out2);
}
#endif

}}}

#pragma GCC diagnostic pop
