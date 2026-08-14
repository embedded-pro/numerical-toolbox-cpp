#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace control_analysis
{
    template<typename T, std::size_t n>
    struct TransferFunction
    {
        static_assert(std::is_floating_point_v<T>, "TransferFunction supports floating-point types");

        std::array<T, n + 1> numerator{};
        std::array<T, n + 1> denominator{};
    };

    template<typename T, std::size_t n>
    class TransferFunctionStateSpace
    {
        static_assert(std::is_floating_point_v<T>, "TransferFunctionStateSpace supports floating-point types");

    public:
        using Realization = math::LinearTimeInvariant<T, n, 1, 1>;
        using TF = TransferFunction<T, n>;
        using StateMatrix = math::Matrix<T, n, n>;
        using InputMatrix = math::Matrix<T, n, 1>;
        using OutputMatrix = math::Matrix<T, 1, n>;
        using FeedMatrix = math::Matrix<T, 1, 1>;

        OPTIMIZE_FOR_SPEED static Realization ToControllableCanonical(const TF& tf);
        OPTIMIZE_FOR_SPEED static Realization ToObservableCanonical(const TF& tf);
        OPTIMIZE_FOR_SPEED static TF ToTransferFunction(const Realization& sys);

    private:
        static std::array<T, n + 1> NormalizeDenominator(const std::array<T, n + 1>& den);
        static std::pair<T, std::array<T, n + 1>> ProperSplit(
            const std::array<T, n + 1>& num, const std::array<T, n + 1>& monicDen);
        static StateMatrix CharacteristicPolynomialCoefficients(const Realization& sys);
        static std::array<T, n + 1> LeVerrierNumerator(
            const Realization& sys, const std::array<T, n + 1>& charPoly);
    };

    ////    Implementation    ////

    template<typename T, std::size_t n>
    std::array<T, n + 1> TransferFunctionStateSpace<T, n>::NormalizeDenominator(
        const std::array<T, n + 1>& den)
    {
        std::array<T, n + 1> result{};
        T leading{ den[0] };
        if (leading == T(0))
            leading = T(1);
        for (std::size_t i{ 0 }; i <= n; ++i)
            result[i] = den[i] / leading;
        return result;
    }

    template<typename T, std::size_t n>
    std::pair<T, std::array<T, n + 1>> TransferFunctionStateSpace<T, n>::ProperSplit(
        const std::array<T, n + 1>& num, const std::array<T, n + 1>& monicDen)
    {
        T leading{ num[0] };
        std::array<T, n + 1> remainder{};
        for (std::size_t i{ 0 }; i <= n; ++i)
            remainder[i] = num[i] - leading * monicDen[i];
        return { leading, remainder };
    }

    template<typename T, std::size_t n>
    OPTIMIZE_FOR_SPEED typename TransferFunctionStateSpace<T, n>::Realization
    TransferFunctionStateSpace<T, n>::ToControllableCanonical(const TF& tf)
    {
        T leading{ tf.denominator[0] };
        if (leading == T(0))
            leading = T(1);
        auto monic{ NormalizeDenominator(tf.denominator) };
        std::array<T, n + 1> scaledNum{};
        for (std::size_t i{ 0 }; i <= n; ++i)
            scaledNum[i] = tf.numerator[i] / leading;
        auto [d, bhat]{ ProperSplit(scaledNum, monic) };

        StateMatrix A{};
        for (std::size_t i{ 0 }; i < n - 1; ++i)
            A.at(i, i + 1) = T(1);
        for (std::size_t j{ 0 }; j < n; ++j)
            A.at(n - 1, j) = -monic[n - j];

        InputMatrix B{};
        B.at(n - 1, 0) = T(1);

        OutputMatrix C{};
        for (std::size_t j{ 0 }; j < n; ++j)
            C.at(0, j) = bhat[n - j];

        FeedMatrix D{};
        D.at(0, 0) = d;

        Realization result{};
        result.A = A;
        result.B = B;
        result.C = C;
        result.D = D;
        return result;
    }

    template<typename T, std::size_t n>
    OPTIMIZE_FOR_SPEED typename TransferFunctionStateSpace<T, n>::Realization
    TransferFunctionStateSpace<T, n>::ToObservableCanonical(const TF& tf)
    {
        auto ccf{ ToControllableCanonical(tf) };
        Realization result{};
        result.A = ccf.A.Transpose();
        result.B = ccf.C.Transpose();
        result.C = ccf.B.Transpose();
        result.D = ccf.D;
        return result;
    }

    template<typename T, std::size_t n>
    OPTIMIZE_FOR_SPEED typename TransferFunctionStateSpace<T, n>::TF
    TransferFunctionStateSpace<T, n>::ToTransferFunction(const Realization& sys)
    {
        using SqMat = math::Matrix<T, n, n>;

        std::array<T, n + 1> charPoly{};
        charPoly[0] = T(1);

        SqMat M{ SqMat::Identity() };

        for (std::size_t k{ 1 }; k <= n; ++k)
        {
            M = sys.A * M;
            charPoly[k] = -M.Trace() / static_cast<T>(k);
            for (std::size_t i{ 0 }; i < n; ++i)
                M.at(i, i) += charPoly[k];
        }

        std::array<T, n + 1> numCoeffs{};
        numCoeffs[0] = sys.D.at(0, 0);

        SqMat N{ SqMat::Identity() };
        for (std::size_t k{ 1 }; k <= n; ++k)
        {
            auto CB{ sys.C * N * sys.B };
            numCoeffs[k] = CB.at(0, 0) + sys.D.at(0, 0) * charPoly[k];
            N = sys.A * N;
            for (std::size_t i{ 0 }; i < n; ++i)
                N.at(i, i) += charPoly[k];
        }

        TF result{};
        result.numerator = numCoeffs;
        result.denominator = charPoly;
        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class TransferFunctionStateSpace<float, 2>;
#endif
}
