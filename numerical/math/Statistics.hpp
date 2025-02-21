#ifndef MATH_STATISTICS_HPP
#define MATH_STATISTICS_HPP

#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"
#include <cmath>

namespace math
{
    template<typename T, size_t Rows, size_t Cols>
    [[nodiscard]] constexpr T Mean(const Matrix<T, Rows, Cols>& data)
    {
        static_assert(detail::is_supported_type_v<T>,
            "Statistical functions only support float or QNumber types");

        T sum{};
        for (size_t i = 0; i < data.size; ++i)
            sum += data.begin()[i];

        return T{ math::ToFloat(sum) / static_cast<float>(data.size) };
    }

    template<typename T, size_t Rows, size_t Cols>
    [[nodiscard]] constexpr T Variance(const Matrix<T, Rows, Cols>& data, bool sample = true)
    {
        static_assert(detail::is_supported_type_v<T>,
            "Statistical functions only support float or QNumber types");

        const T m = Mean(data);
        float sum_sq = 0.0f;

        for (size_t i = 0; i < data.size; ++i)
        {
            float diff = math::ToFloat(data.begin()[i] - m);
            sum_sq += diff * diff;
        }

        return T{ sum_sq / static_cast<float>(sample ? data.size - 1 : data.size) };
    }

    template<typename T, size_t Rows, size_t Cols>
    [[nodiscard]] constexpr T StandardDeviation(const Matrix<T, Rows, Cols>& data, bool sample = true)
    {
        return T{ std::sqrt(math::ToFloat(Variance(data, sample))) };
    }

    template<typename T, size_t Size>
    [[nodiscard]] constexpr T MeanSquaredError(const Vector<T, Size>& actual, const Vector<T, Size>& predicted)
    {
        static_assert(detail::is_supported_type_v<T>,
            "Statistical functions only support float or QNumber types");

        float sum_sq_error = 0.0f;
        for (size_t i = 0; i < Size; ++i)
        {
            float diff = math::ToFloat(actual.at(i, 0) - predicted.at(i, 0));
            sum_sq_error += diff * diff;
        }
        return T{ sum_sq_error / static_cast<float>(Size) };
    }

    template<typename T, size_t Size>
    [[nodiscard]] constexpr T RootMeanSquaredError(const Vector<T, Size>& actual, const Vector<T, Size>& predicted)
    {
        float sum_sq_error = 0.0f;

        for (size_t i = 0; i < Size; ++i)
        {
            float diff = math::ToFloat(actual.at(i, 0) - predicted.at(i, 0));
            sum_sq_error += diff * diff;
        }

        return T{ std::sqrt(sum_sq_error / static_cast<float>(Size)) };
    }

    template<typename T, size_t Size>
    [[nodiscard]] constexpr T MeanAbsoluteError(const Vector<T, Size>& actual, const Vector<T, Size>& predicted)
    {
        static_assert(detail::is_supported_type_v<T>,
            "Statistical functions only support float or QNumber types");

        float sum_abs_error = 0.0f;
        for (size_t i = 0; i < Size; ++i)
            sum_abs_error += std::abs(math::ToFloat(actual.at(i, 0) - predicted.at(i, 0)));

        return T{ sum_abs_error / static_cast<float>(Size) };
    }

    template<typename T, size_t Size>
    [[nodiscard]] constexpr T RSquaredScore(const Vector<T, Size>& actual, const Vector<T, Size>& predicted)
    {
        static_assert(detail::is_supported_type_v<T>,
            "Statistical functions only support float or QNumber types");

        float mean_actual = math::ToFloat(Mean(actual));
        float total_ss = 0.0f;
        float residual_ss = 0.0f;

        for (size_t i = 0; i < Size; ++i)
        {
            float actual_val = math::ToFloat(actual.at(i, 0));
            float pred_val = math::ToFloat(predicted.at(i, 0));
            float diff_mean = actual_val - mean_actual;
            float diff_pred = actual_val - pred_val;
            total_ss += diff_mean * diff_mean;
            residual_ss += diff_pred * diff_pred;
        }

        return T{ 1.0f - (residual_ss / total_ss) };
    }

    template<typename T, size_t Size>
    [[nodiscard]] constexpr Matrix<T, Size, 1> AutoCorrelation(const Vector<T, Size>& data, size_t maxLag)
    {
        static_assert(detail::is_supported_type_v<T>,
            "Statistical functions only support float or QNumber types");

        really_assert(maxLag < Size);

        float mean_val = math::ToFloat(Mean(data));
        float sum_sq = 0.0f;

        for (size_t i = 0; i < Size; ++i)
        {
            float diff = math::ToFloat(data.at(i, 0)) - mean_val;
            sum_sq += diff * diff;
        }

        Matrix<T, Size, 1> result;

        for (size_t lag = 0; lag <= maxLag; ++lag)
        {
            float sum = 0.0f;
            for (size_t t = 0; t < Size - lag; ++t)
                sum += (math::ToFloat(data.at(t, 0)) - mean_val) * (math::ToFloat(data.at(t + lag, 0)) - mean_val);

            if (lag == 0)
                result.at(lag, 0) = T{ 0.9999f };
            else
                result.at(lag, 0) = T{ sum / ((sum_sq / static_cast<float>(Size)) * static_cast<float>(Size - lag)) };
        }

        return result;
    }

    template<typename T, size_t Rows, size_t Cols>
    [[nodiscard]] constexpr Matrix<T, Rows, Cols> ZScore(const Matrix<T, Rows, Cols>& data)
    {
        static_assert(std::is_same_v<T, float>,
            "ZScore function only supports float type");

        const T meanValue = Mean(data);
        const T standardDev = StandardDeviation(data, false);
        really_assert(math::ToFloat(standardDev) > 0.0f);

        Matrix<T, Rows, Cols> result;
        for (size_t i = 0; i < data.size; ++i)
            result.begin()[i] = T{ math::ToFloat(data.begin()[i] - meanValue) / math::ToFloat(standardDev) };

        return result;
    }
}

#endif
