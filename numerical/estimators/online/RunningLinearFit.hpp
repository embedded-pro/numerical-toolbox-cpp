#pragma once

#include "numerical/math/Math.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace estimators
{
    template<typename T>
    class RunningLinearFit
    {
        static_assert(std::is_floating_point_v<T>, "RunningLinearFit supports floating-point types only");

    public:
        void Add(T x, T y, T weight = T{ 1 });
        void Decay(T factor);
        void Reset();

        uint32_t Count() const;
        T Weight() const;
        T MeanX() const;
        T MeanY() const;
        T SumSquaredDeviationsX() const;
        T SumCrossDeviations() const;

        T Slope() const;
        T Intercept() const;
        T RelativeSlopeError() const;

    private:
        uint32_t count{ 0 };
        T weight{};
        T meanX{};
        T meanY{};
        T sumSquaredDeviationsX{};
        T sumCrossDeviations{};
        T sumSquaredDeviationsY{};
    };

    template<typename T>
    void RunningLinearFit<T>::Add(T x, T y, T pointWeight)
    {
        ++count;
        weight += pointWeight;

        const auto deviationX = x - meanX;
        const auto deviationY = y - meanY;
        meanX += pointWeight * deviationX / weight;
        meanY += pointWeight * deviationY / weight;

        sumSquaredDeviationsX += pointWeight * deviationX * (x - meanX);
        sumCrossDeviations += pointWeight * deviationX * (y - meanY);
        sumSquaredDeviationsY += pointWeight * deviationY * (y - meanY);
    }

    template<typename T>
    void RunningLinearFit<T>::Decay(T factor)
    {
        weight *= factor;
        sumSquaredDeviationsX *= factor;
        sumCrossDeviations *= factor;
        sumSquaredDeviationsY *= factor;
    }

    template<typename T>
    void RunningLinearFit<T>::Reset()
    {
        *this = RunningLinearFit{};
    }

    template<typename T>
    uint32_t RunningLinearFit<T>::Count() const
    {
        return count;
    }

    template<typename T>
    T RunningLinearFit<T>::Weight() const
    {
        return weight;
    }

    template<typename T>
    T RunningLinearFit<T>::MeanX() const
    {
        return meanX;
    }

    template<typename T>
    T RunningLinearFit<T>::MeanY() const
    {
        return meanY;
    }

    template<typename T>
    T RunningLinearFit<T>::SumSquaredDeviationsX() const
    {
        return sumSquaredDeviationsX;
    }

    template<typename T>
    T RunningLinearFit<T>::SumCrossDeviations() const
    {
        return sumCrossDeviations;
    }

    template<typename T>
    T RunningLinearFit<T>::Slope() const
    {
        return sumSquaredDeviationsX > T{} ? sumCrossDeviations / sumSquaredDeviationsX : T{};
    }

    template<typename T>
    T RunningLinearFit<T>::Intercept() const
    {
        return meanY - Slope() * meanX;
    }

    template<typename T>
    T RunningLinearFit<T>::RelativeSlopeError() const
    {
        const auto slope = Slope();

        if (count < 3 || slope == T{})
            return std::numeric_limits<T>::infinity();

        const auto residual = std::max(sumSquaredDeviationsY - slope * sumCrossDeviations, T{});
        const auto standardError = math::Sqrt(residual / static_cast<T>(count - 2) / sumSquaredDeviationsX);

        return standardError / math::Abs(slope);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class RunningLinearFit<float>;
#endif
}
