#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <cstddef>
#include <limits>
#include <type_traits>

namespace math
{
    template<typename T>
    using Vector6 = Vector<T, 6>;

    template<typename T>
    using Matrix6 = SquareMatrix<T, 6>;

    template<typename T>
    struct SE3Transform
    {
        static_assert(std::is_floating_point_v<T>, "SE3Transform supports floating-point types only");

        Matrix3<T> rotation{ Matrix3<T>::Identity() };
        Vector3<T> translation{};

        static SE3Transform Identity();
        OPTIMIZE_FOR_SPEED SE3Transform operator*(const SE3Transform& rhs) const;
        SE3Transform Inverse() const;
        OPTIMIZE_FOR_SPEED Vector3<T> Apply(const Vector3<T>& point) const;
        Vector3<T> Rotate(const Vector3<T>& direction) const;
        SquareMatrix<T, 4> Homogeneous() const;
        Matrix6<T> Adjoint() const;
        Vector6<T> TransformTwist(const Vector6<T>& twist) const;
        Vector6<T> TransformWrench(const Vector6<T>& wrench) const;

        OPTIMIZE_FOR_SPEED static SE3Transform Exp(const Vector6<T>& twist, T theta);
        Vector6<T> Log() const;
        static Vector3<T> LogRotation(const Matrix3<T>& r);
        static Vector6<T> PoseError(const SE3Transform& target, const SE3Transform& current);

    private:
        static constexpr T seriesThreshold{ T{ 0.1f } };

        static Vector3<T> Linear(const Vector6<T>& v);
        static Vector3<T> Angular(const Vector6<T>& v);
        static Vector6<T> Stack(const Vector3<T>& linear, const Vector3<T>& angular);
        static Vector3<T> AxisFromSymmetricPart(const Matrix3<T>& r, T cosPhi, const Vector3<T>& sinAxis);
    };

    template<typename T>
    SE3Transform<T> SE3Transform<T>::Identity()
    {
        return {};
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED SE3Transform<T> SE3Transform<T>::operator*(const SE3Transform& rhs) const
    {
        return { rotation * rhs.rotation, rotation * rhs.translation + translation };
    }

    template<typename T>
    SE3Transform<T> SE3Transform<T>::Inverse() const
    {
        const Matrix3<T> transposed{ rotation.Transpose() };
        return { transposed, transposed * translation * T{ -1 } };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED Vector3<T> SE3Transform<T>::Apply(const Vector3<T>& point) const
    {
        return rotation * point + translation;
    }

    template<typename T>
    Vector3<T> SE3Transform<T>::Rotate(const Vector3<T>& direction) const
    {
        return rotation * direction;
    }

    template<typename T>
    SquareMatrix<T, 4> SE3Transform<T>::Homogeneous() const
    {
        SquareMatrix<T, 4> homogeneous{};
        homogeneous.SetBlock(rotation, 0, 0);
        homogeneous.SetBlock(translation, 0, 3);
        homogeneous.at(3, 3) = T{ 1 };
        return homogeneous;
    }

    template<typename T>
    Matrix6<T> SE3Transform<T>::Adjoint() const
    {
        Matrix6<T> adjoint{};
        adjoint.SetBlock(rotation, 0, 0);
        adjoint.SetBlock(SkewSymmetric(translation) * rotation, 0, 3);
        adjoint.SetBlock(rotation, 3, 3);
        return adjoint;
    }

    template<typename T>
    Vector6<T> SE3Transform<T>::TransformTwist(const Vector6<T>& twist) const
    {
        const Vector3<T> angular{ rotation * Angular(twist) };
        return Stack(rotation * Linear(twist) + CrossProduct(translation, angular), angular);
    }

    template<typename T>
    Vector6<T> SE3Transform<T>::TransformWrench(const Vector6<T>& wrench) const
    {
        const Vector3<T> force{ rotation * Linear(wrench) };
        return Stack(force, rotation * Angular(wrench) + CrossProduct(translation, force));
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED SE3Transform<T> SE3Transform<T>::Exp(const Vector6<T>& twist, T theta)
    {
        const Vector3<T> v{ Linear(twist) };
        const Vector3<T> omega{ Angular(twist) };
        const T phi{ theta * VectorNorm(omega) };
        const T phi2{ phi * phi };

        T sinc{ T{ 1 } - phi2 / T{ 6 } + phi2 * phi2 / T{ 120 } };
        T halfVersine{ T{ 0.5f } - phi2 / T{ 24 } + phi2 * phi2 / T{ 720 } };
        T cubic{ T{ 1 } / T{ 6 } - phi2 / T{ 120 } + phi2 * phi2 / T{ 5040 } };
        if (phi >= seriesThreshold)
        {
            const T halfSin{ math::Sin(phi / T{ 2 }) };
            sinc = math::Sin(phi) / phi;
            halfVersine = T{ 2 } * halfSin * halfSin / phi2;
            cubic = (phi - math::Sin(phi)) / (phi2 * phi);
        }

        const Matrix3<T> k{ SkewSymmetric(omega) };
        const Matrix3<T> k2{ k * k };
        const Vector3<T> kv{ k * v };
        const Vector3<T> k2v{ k * kv };

        return { Matrix3<T>::Identity() + k * (theta * sinc) + k2 * (theta * theta * halfVersine),
            v * theta + kv * (theta * theta * halfVersine) + k2v * (theta * theta * theta * cubic) };
    }

    template<typename T>
    Vector6<T> SE3Transform<T>::Log() const
    {
        const Vector3<T> omega{ LogRotation(rotation) };
        const T phi{ VectorNorm(omega) };
        const T phi2{ phi * phi };

        T coefficient{ T{ 1 } / T{ 12 } + phi2 / T{ 720 } + phi2 * phi2 / T{ 30240 } };
        if (phi >= seriesThreshold)
        {
            const T half{ phi / T{ 2 } };
            coefficient = (T{ 1 } - half * math::Cos(half) / math::Sin(half)) / phi2;
        }

        const Vector3<T> kp{ CrossProduct(omega, translation) };
        return Stack(translation - kp * T{ 0.5f } + CrossProduct(omega, kp) * coefficient, omega);
    }

    template<typename T>
    Vector3<T> SE3Transform<T>::LogRotation(const Matrix3<T>& r)
    {
        const Vector3<T> sinAxis{ (r.at(2, 1) - r.at(1, 2)) / T{ 2 }, (r.at(0, 2) - r.at(2, 0)) / T{ 2 }, (r.at(1, 0) - r.at(0, 1)) / T{ 2 } };
        const T sinPhi{ VectorNorm(sinAxis) };
        const T cosPhi{ (r.Trace() - T{ 1 }) / T{ 2 } };
        const T phi{ math::Atan2(sinPhi, cosPhi) };

        if (cosPhi < T{})
            return AxisFromSymmetricPart(r, cosPhi, sinAxis) * phi;

        if (sinPhi > T{})
            return sinAxis * (phi / sinPhi);

        return Vector3<T>{};
    }

    template<typename T>
    Vector6<T> SE3Transform<T>::PoseError(const SE3Transform& target, const SE3Transform& current)
    {
        return Stack(target.translation - current.translation, LogRotation(target.rotation * current.rotation.Transpose()));
    }

    template<typename T>
    Vector3<T> SE3Transform<T>::Linear(const Vector6<T>& v)
    {
        return v.template GetBlock<3, 1>(0, 0);
    }

    template<typename T>
    Vector3<T> SE3Transform<T>::Angular(const Vector6<T>& v)
    {
        return v.template GetBlock<3, 1>(3, 0);
    }

    template<typename T>
    Vector6<T> SE3Transform<T>::Stack(const Vector3<T>& linear, const Vector3<T>& angular)
    {
        Vector6<T> stacked{};
        stacked.SetBlock(linear, 0, 0);
        stacked.SetBlock(angular, 3, 0);
        return stacked;
    }

    template<typename T>
    Vector3<T> SE3Transform<T>::AxisFromSymmetricPart(const Matrix3<T>& r, T cosPhi, const Vector3<T>& sinAxis)
    {
        const T versine{ T{ 1 } - cosPhi };

        std::size_t i{ 0 };
        for (std::size_t k = 1; k < 3; ++k)
            if (r.at(k, k) > r.at(i, i))
                i = k;

        Vector3<T> axis{};
        axis[i] = math::Sqrt(std::max((r.at(i, i) - cosPhi) / versine, T{}));
        for (std::size_t k = 0; k < 3; ++k)
            if (k != i)
                axis[k] = (r.at(i, k) + r.at(k, i)) / (T{ 2 } * versine * axis[i]);

        if (sinAxis[i] < T{})
            axis *= T{ -1 };

        const T norm{ VectorNorm(axis) };
        return axis * (T{ 1 } / norm);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template struct SE3Transform<float>;
#endif
}
