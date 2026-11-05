#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <type_traits>

namespace math
{
    template<typename T>
    using Vector2 = Vector<T, 2>;

    template<typename T>
    using Matrix2 = SquareMatrix<T, 2>;

    template<typename T>
    struct SE2Transform
    {
        static_assert(std::is_floating_point_v<T>, "SE2Transform supports floating-point types only");

        T cosine{ 1 };
        T sine{ 0 };
        Vector2<T> translation{};

        static SE2Transform Identity();
        static SE2Transform FromPose(T x, T y, T theta);

        Vector3<T> Pose() const;
        T Heading() const;
        Matrix2<T> Rotation() const;

        [[nodiscard]] OPTIMIZE_FOR_SPEED friend SE2Transform operator*(const SE2Transform& lhs, const SE2Transform& rhs)
        {
            return { lhs.cosine * rhs.cosine - lhs.sine * rhs.sine, lhs.sine * rhs.cosine + lhs.cosine * rhs.sine, lhs.Apply(rhs.translation) };
        }

        SE2Transform Inverse() const;
        OPTIMIZE_FOR_SPEED Vector2<T> Apply(const Vector2<T>& point) const;
        Vector2<T> Rotate(const Vector2<T>& direction) const;
        SquareMatrix<T, 3> Homogeneous() const;
        Matrix3<T> Adjoint() const;
        Vector3<T> TransformTwist(const Vector3<T>& twist) const;
        Vector3<T> TransformWrench(const Vector3<T>& wrench) const;
        SE2Transform Normalized() const;
        SE3Transform<T> ToSE3() const;

        OPTIMIZE_FOR_SPEED static SE2Transform Exp(const Vector3<T>& twist, T theta);
        Vector3<T> Log() const;
        static Vector3<T> PoseError(const SE2Transform& target, const SE2Transform& current);

    private:
        static constexpr T seriesThreshold{ T{ 0.1f } };
    };

    template<typename T>
    SE2Transform<T> SE2Transform<T>::Identity()
    {
        return {};
    }

    template<typename T>
    SE2Transform<T> SE2Transform<T>::FromPose(T x, T y, T theta)
    {
        return { math::Cos(theta), math::Sin(theta), Vector2<T>{ x, y } };
    }

    template<typename T>
    Vector3<T> SE2Transform<T>::Pose() const
    {
        return Vector3<T>{ translation[0], translation[1], Heading() };
    }

    template<typename T>
    T SE2Transform<T>::Heading() const
    {
        return math::Atan2(sine, cosine);
    }

    template<typename T>
    Matrix2<T> SE2Transform<T>::Rotation() const
    {
        return Matrix2<T>{ { cosine, -sine }, { sine, cosine } };
    }

    template<typename T>
    SE2Transform<T> SE2Transform<T>::Inverse() const
    {
        return { cosine, -sine, Vector2<T>{ -(cosine * translation[0] + sine * translation[1]), sine * translation[0] - cosine * translation[1] } };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED Vector2<T> SE2Transform<T>::Apply(const Vector2<T>& point) const
    {
        return Vector2<T>{ cosine * point[0] - sine * point[1] + translation[0], sine * point[0] + cosine * point[1] + translation[1] };
    }

    template<typename T>
    Vector2<T> SE2Transform<T>::Rotate(const Vector2<T>& direction) const
    {
        return Vector2<T>{ cosine * direction[0] - sine * direction[1], sine * direction[0] + cosine * direction[1] };
    }

    template<typename T>
    SquareMatrix<T, 3> SE2Transform<T>::Homogeneous() const
    {
        return SquareMatrix<T, 3>{ { cosine, -sine, translation[0] }, { sine, cosine, translation[1] }, { T{}, T{}, T{ 1 } } };
    }

    template<typename T>
    Matrix3<T> SE2Transform<T>::Adjoint() const
    {
        return Matrix3<T>{ { cosine, -sine, translation[1] }, { sine, cosine, -translation[0] }, { T{}, T{}, T{ 1 } } };
    }

    template<typename T>
    Vector3<T> SE2Transform<T>::TransformTwist(const Vector3<T>& twist) const
    {
        return Adjoint() * twist;
    }

    template<typename T>
    Vector3<T> SE2Transform<T>::TransformWrench(const Vector3<T>& wrench) const
    {
        const Vector2<T> force{ Rotate(Vector2<T>{ wrench[0], wrench[1] }) };
        return Vector3<T>{ force[0], force[1], wrench[2] + translation[0] * force[1] - translation[1] * force[0] };
    }

    template<typename T>
    SE2Transform<T> SE2Transform<T>::Normalized() const
    {
        const T scale{ T{ 1 } / math::Hypot(cosine, sine) };
        return { cosine * scale, sine * scale, translation };
    }

    template<typename T>
    SE3Transform<T> SE2Transform<T>::ToSE3() const
    {
        return SE3Transform<T>{ Matrix3<T>{ { cosine, -sine, T{} }, { sine, cosine, T{} }, { T{}, T{}, T{ 1 } } }, Vector3<T>{ translation[0], translation[1], T{} } };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED SE2Transform<T> SE2Transform<T>::Exp(const Vector3<T>& twist, T theta)
    {
        const T phi{ twist[2] * theta };
        const T phi2{ phi * phi };

        T sinc{ T{ 1 } - phi2 / T{ 6 } + phi2 * phi2 / T{ 120 } };
        T versinc{ phi / T{ 2 } - phi * phi2 / T{ 24 } + phi * phi2 * phi2 / T{ 720 } };
        if (math::Abs(phi) >= seriesThreshold)
        {
            const T halfSin{ math::Sin(phi / T{ 2 }) };
            sinc = math::Sin(phi) / phi;
            versinc = T{ 2 } * halfSin * halfSin / phi;
        }

        const T ux{ twist[0] * theta };
        const T uy{ twist[1] * theta };
        return { math::Cos(phi), math::Sin(phi), Vector2<T>{ sinc * ux - versinc * uy, versinc * ux + sinc * uy } };
    }

    template<typename T>
    Vector3<T> SE2Transform<T>::Log() const
    {
        const T phi{ Heading() };
        const T half{ phi / T{ 2 } };
        const T phi2{ phi * phi };

        T halfCot{ T{ 1 } - phi2 / T{ 12 } - phi2 * phi2 / T{ 720 } };
        if (math::Abs(phi) >= seriesThreshold)
            halfCot = half * math::Cos(half) / math::Sin(half);

        return Vector3<T>{ halfCot * translation[0] + half * translation[1], -half * translation[0] + halfCot * translation[1], phi };
    }

    template<typename T>
    Vector3<T> SE2Transform<T>::PoseError(const SE2Transform& target, const SE2Transform& current)
    {
        const T heading{ math::Atan2(target.sine * current.cosine - target.cosine * current.sine, target.cosine * current.cosine + target.sine * current.sine) };
        return Vector3<T>{ target.translation[0] - current.translation[0], target.translation[1] - current.translation[1], heading };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template struct SE2Transform<float>;
#endif
}
