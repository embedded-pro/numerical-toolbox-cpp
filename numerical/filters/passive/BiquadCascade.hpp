#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace filters::passive
{
    template<typename T>
    struct BiquadCoeffs
    {
        static_assert(std::is_floating_point_v<T>, "BiquadCoeffs supports floating-point types");

        T b0{};
        T b1{};
        T b2{};
        T a1{};
        T a2{};
    };

    template<typename T>
    class Biquad
    {
        static_assert(std::is_floating_point_v<T>, "Biquad supports floating-point types");

    public:
        explicit Biquad(BiquadCoeffs<T> coeffs) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T x) noexcept;
        void Reset() noexcept;

        static BiquadCoeffs<T> LowPass(T fc, T fs, T Q) noexcept;
        static BiquadCoeffs<T> HighPass(T fc, T fs, T Q) noexcept;
        static BiquadCoeffs<T> BandPass(T fc, T fs, T Q) noexcept;
        static BiquadCoeffs<T> Notch(T fc, T fs, T Q) noexcept;
        static BiquadCoeffs<T> Peaking(T fc, T fs, T Q, T gainDb) noexcept;

    private:
        BiquadCoeffs<T> c{};
        T z1{};
        T z2{};
    };

    template<typename T, std::size_t Sections>
    class BiquadCascade
    {
        static_assert(std::is_floating_point_v<T>, "BiquadCascade supports floating-point types");
        static_assert(Sections > 0, "BiquadCascade must have at least one section");

    public:
        explicit BiquadCascade(std::array<BiquadCoeffs<T>, Sections> coeffs) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T x) noexcept;
        void Reset() noexcept;

    private:
        std::array<Biquad<T>, Sections> stages;
    };

    ////    Biquad Implementation    ////

    template<typename T>
    Biquad<T>::Biquad(BiquadCoeffs<T> coeffs) noexcept
        : c{ coeffs }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED T Biquad<T>::Filter(T x) noexcept
    {
        const T y{ c.b0 * x + z1 };
        z1 = c.b1 * x - c.a1 * y + z2;
        z2 = c.b2 * x - c.a2 * y;
        return y;
    }

    template<typename T>
    void Biquad<T>::Reset() noexcept
    {
        z1 = T{};
        z2 = T{};
    }

    template<typename T>
    BiquadCoeffs<T> Biquad<T>::LowPass(T fc, T fs, T Q) noexcept
    {
        const T w0{ T{ 2 } * std::numbers::pi_v<T> * fc / fs };
        const T cw{ math::Cos(w0) };
        const T alpha{ math::Sin(w0) / (T{ 2 } * Q) };
        const T a0{ T{ 1 } + alpha };
        return BiquadCoeffs<T>{
            (T{ 1 } - cw) / (T{ 2 } * a0),
            (T{ 1 } - cw) / a0,
            (T{ 1 } - cw) / (T{ 2 } * a0),
            (T{ -2 } * cw) / a0,
            (T{ 1 } - alpha) / a0
        };
    }

    template<typename T>
    BiquadCoeffs<T> Biquad<T>::HighPass(T fc, T fs, T Q) noexcept
    {
        const T w0{ T{ 2 } * std::numbers::pi_v<T> * fc / fs };
        const T cw{ math::Cos(w0) };
        const T alpha{ math::Sin(w0) / (T{ 2 } * Q) };
        const T a0{ T{ 1 } + alpha };
        return BiquadCoeffs<T>{
            (T{ 1 } + cw) / (T{ 2 } * a0),
            -(T{ 1 } + cw) / a0,
            (T{ 1 } + cw) / (T{ 2 } * a0),
            (T{ -2 } * cw) / a0,
            (T{ 1 } - alpha) / a0
        };
    }

    template<typename T>
    BiquadCoeffs<T> Biquad<T>::BandPass(T fc, T fs, T Q) noexcept
    {
        const T w0{ T{ 2 } * std::numbers::pi_v<T> * fc / fs };
        const T alpha{ math::Sin(w0) / (T{ 2 } * Q) };
        const T cw{ math::Cos(w0) };
        const T a0{ T{ 1 } + alpha };
        return BiquadCoeffs<T>{
            (math::Sin(w0) / T{ 2 }) / a0,
            T{ 0 },
            -(math::Sin(w0) / T{ 2 }) / a0,
            (T{ -2 } * cw) / a0,
            (T{ 1 } - alpha) / a0
        };
    }

    template<typename T>
    BiquadCoeffs<T> Biquad<T>::Notch(T fc, T fs, T Q) noexcept
    {
        const T w0{ T{ 2 } * std::numbers::pi_v<T> * fc / fs };
        const T cw{ math::Cos(w0) };
        const T alpha{ math::Sin(w0) / (T{ 2 } * Q) };
        const T a0{ T{ 1 } + alpha };
        return BiquadCoeffs<T>{
            T{ 1 } / a0,
            (T{ -2 } * cw) / a0,
            T{ 1 } / a0,
            (T{ -2 } * cw) / a0,
            (T{ 1 } - alpha) / a0
        };
    }

    template<typename T>
    BiquadCoeffs<T> Biquad<T>::Peaking(T fc, T fs, T Q, T gainDb) noexcept
    {
        const T A{ math::Pow(T{ 10 }, gainDb / T{ 40 }) };
        const T w0{ T{ 2 } * std::numbers::pi_v<T> * fc / fs };
        const T cw{ math::Cos(w0) };
        const T alpha{ math::Sin(w0) / (T{ 2 } * Q) };
        const T a0{ T{ 1 } + alpha / A };
        return BiquadCoeffs<T>{
            (T{ 1 } + alpha * A) / a0,
            (T{ -2 } * cw) / a0,
            (T{ 1 } - alpha * A) / a0,
            (T{ -2 } * cw) / a0,
            (T{ 1 } - alpha / A) / a0
        };
    }

    ////    BiquadCascade Implementation    ////

    template<typename T, std::size_t Sections>
    BiquadCascade<T, Sections>::BiquadCascade(std::array<BiquadCoeffs<T>, Sections> coeffs) noexcept
        : stages{ [&]()
            {
                std::array<Biquad<T>, Sections> arr{ [&]<std::size_t... Is>(std::index_sequence<Is...>) -> std::array<Biquad<T>, Sections>
                    {
                        return { Biquad<T>{ coeffs[Is] }... };
                    }(std::make_index_sequence<Sections>{}) };
                return arr;
            }() }
    {}

    template<typename T, std::size_t Sections>
    OPTIMIZE_FOR_SPEED T BiquadCascade<T, Sections>::Filter(T x) noexcept
    {
        for (auto& stage : stages)
            x = stage.Filter(x);
        return x;
    }

    template<typename T, std::size_t Sections>
    void BiquadCascade<T, Sections>::Reset() noexcept
    {
        for (auto& stage : stages)
            stage.Reset();
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template struct BiquadCoeffs<float>;
    extern template class Biquad<float>;
    extern template class BiquadCascade<float, 2>;
#endif
}
