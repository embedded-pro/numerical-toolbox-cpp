#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixExponential.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <optional>
#include <type_traits>

namespace control_analysis
{
    enum class DiscretizationMethod
    {
        ZeroOrderHold,
        Tustin,
        ForwardEuler,
        BackwardEuler
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    class ContinuousToDiscrete
    {
        static_assert(std::is_floating_point_v<T>, "ContinuousToDiscrete supports floating-point types");

    public:
        using SystemType = math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>;

        ContinuousToDiscrete() = default;

        OPTIMIZE_FOR_SPEED std::optional<SystemType> Convert(const SystemType& sys, T ts, DiscretizationMethod method);

    private:
        [[no_unique_address]] math::MatrixExponential<T, StateSize + InputSize> expm{};

        static std::optional<math::SquareMatrix<T, StateSize>> Invert(const math::SquareMatrix<T, StateSize>& a);

        std::optional<SystemType> Zoh(const SystemType& sys, T ts);
        std::optional<SystemType> Bilinear(const SystemType& sys, T ts);
        SystemType ForwardEuler(const SystemType& sys, T ts);
        std::optional<SystemType> Backward(const SystemType& sys, T ts);
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    std::optional<math::SquareMatrix<T, StateSize>>
    ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::Invert(const math::SquareMatrix<T, StateSize>& a)
    {
        solvers::LuDecomposition<T, StateSize> lu{};

        if (!lu.Decompose(a))
            return std::nullopt;

        return lu.Inverse();
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    OPTIMIZE_FOR_SPEED std::optional<typename ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::SystemType>
    ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::Convert(const SystemType& sys, T ts, DiscretizationMethod method)
    {
        if (!math::IsFinite(ts) || ts <= T{})
            return std::nullopt;

        switch (method)
        {
            case DiscretizationMethod::ZeroOrderHold:
                return Zoh(sys, ts);
            case DiscretizationMethod::Tustin:
                return Bilinear(sys, ts);
            case DiscretizationMethod::ForwardEuler:
                return ForwardEuler(sys, ts);
            case DiscretizationMethod::BackwardEuler:
                return Backward(sys, ts);
        }
        return Zoh(sys, ts);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    std::optional<typename ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::SystemType>
    ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::Zoh(const SystemType& sys, T ts)
    {
        math::SquareMatrix<T, StateSize + InputSize> augmented{};

        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < StateSize; ++j)
                augmented.at(i, j) = sys.A.at(i, j) * ts;

        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < InputSize; ++j)
                augmented.at(i, StateSize + j) = sys.B.at(i, j) * ts;

        const auto expM = expm.Compute(augmented);

        if (!expM)
            return std::nullopt;

        SystemType result{};
        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < StateSize; ++j)
                result.A.at(i, j) = expM->at(i, j);

        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < InputSize; ++j)
                result.B.at(i, j) = expM->at(i, StateSize + j);

        result.C = sys.C;
        result.D = sys.D;
        return result;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    std::optional<typename ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::SystemType>
    ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::Bilinear(const SystemType& sys, T ts)
    {
        const T alpha{ T{ 2 } / ts };
        const auto identity = math::SquareMatrix<T, StateSize>::Identity();
        const auto alphaI = identity * alpha;
        const auto lhs = alphaI - sys.A;
        const auto inverse = Invert(lhs);

        if (!inverse)
            return std::nullopt;

        const auto& P = *inverse;
        const auto Ad = P * (alphaI + sys.A);
        const auto Bd = P * sys.B * T{ 2 };
        const auto Cd = sys.C * P * alpha;
        const auto Dd = sys.D + sys.C * P * sys.B;

        SystemType result{};
        result.A = Ad;
        result.B = Bd;
        result.C = Cd;
        result.D = Dd;
        return result;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    typename ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::SystemType
    ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::ForwardEuler(const SystemType& sys, T ts)
    {
        SystemType result{};
        const auto identity = math::SquareMatrix<T, StateSize>::Identity();
        result.A = identity + sys.A * ts;
        result.B = sys.B * ts;
        result.C = sys.C;
        result.D = sys.D;
        return result;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    std::optional<typename ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::SystemType>
    ContinuousToDiscrete<T, StateSize, InputSize, OutputSize>::Backward(const SystemType& sys, T ts)
    {
        const auto identity = math::SquareMatrix<T, StateSize>::Identity();
        const auto lhs = identity - sys.A * ts;
        const auto inverse = Invert(lhs);

        if (!inverse)
            return std::nullopt;

        const auto& P = *inverse;
        const auto PB = P * sys.B;
        const auto CP = sys.C * P;

        SystemType result{};
        result.A = P;
        result.B = PB * ts;
        result.C = CP;
        result.D = sys.D + CP * sys.B * ts;
        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ContinuousToDiscrete<float, 2, 1, 1>;
#endif
}
