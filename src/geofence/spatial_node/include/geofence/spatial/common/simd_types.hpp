/// @file simd_types.hpp
/// @brief SIMD type aliases for xsimd compatibility across versions.
///
/// xsimd 7.x uses batch<T, N> where N is the SIMD width.
/// xsimd 8+ uses batch<T, Arch> with a default architecture.
/// This header provides a single alias that works with the installed version.

#pragma once

#ifdef USE_SIMD
#include <xsimd/xsimd.hpp>

namespace rises::geofence::simd {

// xsimd 7.x: XSIMD_BATCH_FLOAT_SIZE is defined in xsimd/types/xsimd_traits.hpp
// and resolves to 4 (SSE/NEON), 8 (AVX), or 16 (AVX-512) based on the target.
#ifdef XSIMD_BATCH_FLOAT_SIZE
    using float_batch = xsimd::batch<float, XSIMD_BATCH_FLOAT_SIZE>;
    using double_batch = xsimd::batch<double, XSIMD_BATCH_DOUBLE_SIZE>;
#else
    // xsimd 8+: default_arch selects the best available instruction set.
    using float_batch = xsimd::batch<float, xsimd::default_arch>;
    using double_batch = xsimd::batch<double, xsimd::default_arch>;
#endif

using float_bool_batch = decltype(float_batch() == float_batch());
constexpr std::size_t float_simd_size = float_batch::size;
constexpr std::size_t simd_alignment = float_simd_size * sizeof(float);

} // namespace rises::geofence::simd

#endif // USE_SIMD
