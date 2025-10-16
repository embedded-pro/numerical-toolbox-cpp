#pragma once

#if defined(__GNUC__) && !defined(__clang__)
#define OPTIMIZE_FOR_SPEED __attribute__((hot, optimize("-O3"), optimize("-ffast-math"), flatten))
#elif defined(__clang__)
#define OPTIMIZE_FOR_SPEED __attribute__((hot, flatten))
#elif defined(_MSC_VER)
#define OPTIMIZE_FOR_SPEED
#else
#define OPTIMIZE_FOR_SPEED
#endif

#if defined(__GNUC__) || defined(__clang__)
#define ALWAYS_INLINE_HOT __attribute__((always_inline, hot)) inline
#elif defined(_MSC_VER)
#define ALWAYS_INLINE_HOT __forceinline
#else
#define ALWAYS_INLINE_HOT inline
#endif

#if defined(__GNUC__) || defined(__clang__)
#define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined(_MSC_VER)
#define ALWAYS_INLINE __forceinline
#else
#define ALWAYS_INLINE inline
#endif
