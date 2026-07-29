#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include <cmath>
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace analysis
{
    template<typename T>
    class GoertzelAlgorithm
    {
        static_assert(std::is_floating_point_v<T>, "GoertzelAlgorithm supports floating-point types only");

    public:
        GoertzelAlgorithm(std::size_t k, std::size_t blockLength);
        GoertzelAlgorithm(T targetHz, T sampleHz, std::size_t blockLength);

        OPTIMIZE_FOR_SPEED void Push(T x);
        bool Ready() const;
        math::Complex<T> Result() const;
        T Magnitude() const;
        void Reset();

        static T Coefficient(std::size_t k, std::size_t blockLength);

    private:
        T coeff;
        T cosine;
        T sine;
        std::size_t blockSize;
        T s1{ T{ 0 } };
        T s2{ T{ 0 } };
        std::size_t sampleCount{ 0 };
    };

    template<typename T>
    GoertzelAlgorithm<T>::GoertzelAlgorithm(std::size_t k, std::size_t blockLength)
        : coeff{ T{ 2 } * std::cos(T{ 2 } * std::numbers::pi_v<T> * static_cast<T>(k) / static_cast<T>(blockLength)) }
        , cosine{ std::cos(T{ 2 } * std::numbers::pi_v<T> * static_cast<T>(k) / static_cast<T>(blockLength)) }
        , sine{ std::sin(T{ 2 } * std::numbers::pi_v<T> * static_cast<T>(k) / static_cast<T>(blockLength)) }
        , blockSize{ blockLength }
    {}

    template<typename T>
    GoertzelAlgorithm<T>::GoertzelAlgorithm(T targetHz, T sampleHz, std::size_t blockLength)
        : GoertzelAlgorithm(
              static_cast<std::size_t>(targetHz / sampleHz * static_cast<T>(blockLength) + T{ 0.5 }),
              blockLength)
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED void GoertzelAlgorithm<T>::Push(T x)
    {
        T s0{ x + coeff * s1 - s2 };
        s2 = s1;
        s1 = s0;
        ++sampleCount;
    }

    template<typename T>
    bool GoertzelAlgorithm<T>::Ready() const
    {
        return sampleCount >= blockSize;
    }

    template<typename T>
    math::Complex<T> GoertzelAlgorithm<T>::Result() const
    {
        T real{ s1 - s2 * cosine };
        T imag{ s2 * sine };
        return math::Complex<T>{ real, imag };
    }

    template<typename T>
    T GoertzelAlgorithm<T>::Magnitude() const
    {
        return std::sqrt(s1 * s1 + s2 * s2 - coeff * s1 * s2);
    }

    template<typename T>
    void GoertzelAlgorithm<T>::Reset()
    {
        s1 = T{ 0 };
        s2 = T{ 0 };
        sampleCount = 0;
    }

    template<typename T>
    T GoertzelAlgorithm<T>::Coefficient(std::size_t k, std::size_t blockLength)
    {
        return T{ 2 } * std::cos(T{ 2 } * std::numbers::pi_v<T> * static_cast<T>(k) / static_cast<T>(blockLength));
    }
}

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
namespace analysis
{
    extern template class GoertzelAlgorithm<float>;
}
#endif
