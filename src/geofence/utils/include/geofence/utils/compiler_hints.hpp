#pragma once

// ============================================================================
// Branch Prediction Hints
// ============================================================================

#if defined(__GNUC__) || defined(__clang__)
    #define LIKELY(x)   __builtin_expect(!!(x), 1)
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
    #define LIKELY(x)   (x)
    #define UNLIKELY(x) (x)
#endif

// ============================================================================
// Cache Prefetch Hints
// ============================================================================

/**
 * @brief Prefetch data into cache for future access
 * 
 * Locality hints:
 * - 0: No temporal locality (data used once)
 * - 1: Low temporal locality
 * - 2: Moderate temporal locality
 * - 3: High temporal locality (data reused soon)
 * 
 * Usage:
 * @code
 * for (std::size_t i = 0; i < items.size(); ++i) {
 *     PREFETCH_READ(&items[i + 1], 3);  // Prefetch next item
 *     process(items[i]);
 * }
 * @endcode
 */
#if defined(__GNUC__) || defined(__clang__)
    // Read prefetch: rw=0, locality (0-3)
    #define PREFETCH_READ(addr, locality) __builtin_prefetch((addr), 0, (locality))
    // Write prefetch: rw=1, locality (0-3)
    #define PREFETCH_WRITE(addr, locality) __builtin_prefetch((addr), 1, (locality))
#elif defined(_MSC_VER)
    #include <intrin.h>
    #define PREFETCH_READ(addr, locality) _mm_prefetch((const char*)(addr), _MM_HINT_T0)
    #define PREFETCH_WRITE(addr, locality) _mm_prefetch((const char*)(addr), _MM_HINT_T0)
#else
    #define PREFETCH_READ(addr, locality) ((void)0)
    #define PREFETCH_WRITE(addr, locality) ((void)0)
#endif
