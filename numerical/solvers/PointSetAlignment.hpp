#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "numerical/solvers/SingularValueDecomposition.hpp"
#include <cstddef>
#include <optional>
#include <span>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Dim>
    struct Alignment
    {
        static_assert(std::is_floating_point_v<T>, "Alignment supports floating-point types");
        static_assert(Dim == 2 || Dim == 3, "Alignment supports two or three dimensions");

        math::SquareMatrix<T, Dim> rotation{ math::SquareMatrix<T, Dim>::Identity() };
        math::Vector<T, Dim> translation{};
        T scale{ 1 };
        T rms{};
    };

    template<typename T, std::size_t Dim>
    class PointSetAlignment
    {
        static_assert(std::is_floating_point_v<T>, "PointSetAlignment supports floating-point types");
        static_assert(Dim == 2 || Dim == 3, "PointSetAlignment supports two or three dimensions");

    public:
        using Point = math::Vector<T, Dim>;
        using Rotation = math::SquareMatrix<T, Dim>;

        OPTIMIZE_FOR_SPEED static std::optional<Alignment<T, Dim>> Align(std::span<const Point> source, std::span<const Point> target, bool estimateScale = false,
            std::span<const T> weights = {});

    private:
        struct Moments
        {
            Point sourceMean;
            Point targetMean;
            T sourceSpread;
            T targetSpread;
            Rotation crossCovariance;
            T totalWeight;
        };

        struct Fit
        {
            Rotation rotation;
            T trace;
        };

        static constexpr T tolerance{ T{ 1e-5f } };

        static T Weight(std::span<const T> weights, std::size_t index);
        static std::optional<Moments> Accumulate(std::span<const Point> source, std::span<const Point> target, std::span<const T> weights);
        static std::optional<Fit> SolveRotation(const Moments& moments);
        static T Determinant(const math::SquareMatrix<T, 3>& m);
        static T Rms(std::span<const Point> source, std::span<const Point> target, std::span<const T> weights, const Alignment<T, Dim>& alignment, T totalWeight);
    };

    template<typename T, std::size_t Dim>
    OPTIMIZE_FOR_SPEED std::optional<Alignment<T, Dim>> PointSetAlignment<T, Dim>::Align(std::span<const Point> source, std::span<const Point> target, bool estimateScale,
        std::span<const T> weights)
    {
        const auto moments{ Accumulate(source, target, weights) };
        if (!moments)
            return std::nullopt;

        const auto fit{ SolveRotation(*moments) };
        if (!fit)
            return std::nullopt;

        Alignment<T, Dim> alignment{};
        alignment.rotation = fit->rotation;
        alignment.scale = estimateScale ? fit->trace / moments->sourceSpread : T{ 1 };
        alignment.translation = moments->targetMean - fit->rotation * moments->sourceMean * alignment.scale;
        alignment.rms = Rms(source, target, weights, alignment, moments->totalWeight);
        return alignment;
    }

    template<typename T, std::size_t Dim>
    T PointSetAlignment<T, Dim>::Weight(std::span<const T> weights, std::size_t index)
    {
        return weights.empty() ? T{ 1 } : weights[index];
    }

    template<typename T, std::size_t Dim>
    auto PointSetAlignment<T, Dim>::Accumulate(std::span<const Point> source, std::span<const Point> target, std::span<const T> weights) -> std::optional<Moments>
    {
        if (source.size() != target.size() || (!weights.empty() && weights.size() != source.size()))
            return std::nullopt;

        Moments moments{};
        std::size_t supported{ 0 };
        for (std::size_t i = 0; i < source.size(); ++i)
        {
            const T weight{ Weight(weights, i) };
            supported += weight > T{} ? 1 : 0;
            moments.totalWeight += weight;
            moments.sourceMean = moments.sourceMean + source[i] * weight;
            moments.targetMean = moments.targetMean + target[i] * weight;
        }

        if (supported < Dim)
            return std::nullopt;

        moments.sourceMean = moments.sourceMean * (T{ 1 } / moments.totalWeight);
        moments.targetMean = moments.targetMean * (T{ 1 } / moments.totalWeight);

        for (std::size_t i = 0; i < source.size(); ++i)
        {
            const T weight{ Weight(weights, i) / moments.totalWeight };
            const Point a{ source[i] - moments.sourceMean };
            const Point b{ target[i] - moments.targetMean };
            moments.sourceSpread += weight * math::DotProduct(a, a);
            moments.targetSpread += weight * math::DotProduct(b, b);
            moments.crossCovariance = moments.crossCovariance + b * a.Transpose() * weight;
        }

        return moments;
    }

    template<typename T, std::size_t Dim>
    auto PointSetAlignment<T, Dim>::SolveRotation(const Moments& moments) -> std::optional<Fit>
    {
        const auto& cross{ moments.crossCovariance };
        if constexpr (Dim == 2)
        {
            const T cosine{ cross.at(0, 0) + cross.at(1, 1) };
            const T sine{ cross.at(1, 0) - cross.at(0, 1) };
            const T magnitude{ math::Hypot(cosine, sine) };
            if (magnitude <= tolerance * math::Sqrt(moments.sourceSpread * moments.targetSpread))
                return std::nullopt;

            const T angle{ math::Atan2(sine, cosine) };
            return Fit{ Rotation{ { math::Cos(angle), -math::Sin(angle) }, { math::Sin(angle), math::Cos(angle) } }, magnitude };
        }
        else
        {
            SingularValueDecomposition<T, 3, 3> svd{};
            const bool decomposed{ svd.Decompose(cross) };
            const auto& singular{ svd.SingularValues() };
            if (!decomposed || singular[1] <= tolerance * singular[0])
                return std::nullopt;

            const T sign{ Determinant(svd.U()) * Determinant(svd.V()) < T{} ? T{ -1 } : T{ 1 } };
            const Rotation flip{ { T{ 1 }, T{}, T{} }, { T{}, T{ 1 }, T{} }, { T{}, T{}, sign } };
            return Fit{ svd.U() * flip * svd.V().Transpose(), singular[0] + singular[1] + sign * singular[2] };
        }
    }

    template<typename T, std::size_t Dim>
    T PointSetAlignment<T, Dim>::Determinant(const math::SquareMatrix<T, 3>& m)
    {
        return m.at(0, 0) * (m.at(1, 1) * m.at(2, 2) - m.at(1, 2) * m.at(2, 1)) - m.at(0, 1) * (m.at(1, 0) * m.at(2, 2) - m.at(1, 2) * m.at(2, 0)) +
               m.at(0, 2) * (m.at(1, 0) * m.at(2, 1) - m.at(1, 1) * m.at(2, 0));
    }

    template<typename T, std::size_t Dim>
    T PointSetAlignment<T, Dim>::Rms(std::span<const Point> source, std::span<const Point> target, std::span<const T> weights, const Alignment<T, Dim>& alignment, T totalWeight)
    {
        T sum{};
        for (std::size_t i = 0; i < source.size(); ++i)
        {
            const Point error{ target[i] - (alignment.rotation * source[i] * alignment.scale + alignment.translation) };
            sum += Weight(weights, i) * math::DotProduct(error, error);
        }
        return math::Sqrt(sum / totalWeight);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PointSetAlignment<float, 2>;
    extern template class PointSetAlignment<float, 3>;
#endif
}
