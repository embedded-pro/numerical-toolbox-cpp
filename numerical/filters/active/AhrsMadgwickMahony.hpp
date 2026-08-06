#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Quaternion.hpp"
#include "numerical/math/Math.hpp"
#include <type_traits>

namespace filters
{
    enum class AhrsMode
    {
        Madgwick,
        Mahony
    };

    template<typename T, AhrsMode M>
    class AhrsFilter
    {
        static_assert(std::is_floating_point_v<T>, "AhrsFilter supports floating-point types only");

    public:
        AhrsFilter(T gain, T samplePeriod);

        OPTIMIZE_FOR_SPEED void UpdateImu(math::Vector3<T> gyro, math::Vector3<T> accel);
        OPTIMIZE_FOR_SPEED void UpdateMarg(math::Vector3<T> gyro, math::Vector3<T> accel, math::Vector3<T> mag);

        math::Quaternion<T> Orientation() const;
        math::Vector3<T> Euler() const;
        void Reset();

    private:
        math::Quaternion<T> q{ math::Quaternion<T>::Identity() };
        T Ts;
        T beta;
        T Kp;
        T Ki;
        math::Vector3<T> integralFb{ { T{} }, { T{} }, { T{} } };

        static math::Vector3<T> GravityFromQuaternion(const math::Quaternion<T>& quat);
        static math::Vector3<T> NorthFromQuaternion(const math::Quaternion<T>& quat, T bx, T bz);

        static math::Quaternion<T> GradientGravity(const math::Quaternion<T>& quat, const math::Vector3<T>& a);
        static math::Quaternion<T> GradientMag(const math::Quaternion<T>& quat, const math::Vector3<T>& m, T bx, T bz);

        static T SafeInvSqrt(T x);
        static math::Vector3<T> NormalizeVec(const math::Vector3<T>& v, T norm);

        void IntegrateGyro(const math::Vector3<T>& gyro);
        void MahonyStep(const math::Vector3<T>& gyro, const math::Vector3<T>& e);
        void MadgwickStep(const math::Vector3<T>& gyro, const math::Quaternion<T>& grad);
    };

    ////    Implementation    ////

    template<typename T, AhrsMode M>
    AhrsFilter<T, M>::AhrsFilter(T gain, T samplePeriod)
        : Ts{ samplePeriod }
        , beta{ (M == AhrsMode::Madgwick) ? gain : T{} }
        , Kp{ (M == AhrsMode::Mahony) ? gain : T{} }
        , Ki{ (M == AhrsMode::Mahony) ? T(0.01) : T{} }
    {}

    template<typename T, AhrsMode M>
    T AhrsFilter<T, M>::SafeInvSqrt(T x)
    {
        return (x > T{}) ? T(1) / math::Sqrt(x) : T{};
    }

    template<typename T, AhrsMode M>
    math::Vector3<T> AhrsFilter<T, M>::NormalizeVec(const math::Vector3<T>& v, T norm)
    {
        T inv{ SafeInvSqrt(norm * norm) };
        return math::Vector3<T>{ { v.at(0, 0) * inv }, { v.at(1, 0) * inv }, { v.at(2, 0) * inv } };
    }

    template<typename T, AhrsMode M>
    math::Vector3<T> AhrsFilter<T, M>::GravityFromQuaternion(const math::Quaternion<T>& quat)
    {
        T qw{ quat.w };
        T qx{ quat.x };
        T qy{ quat.y };
        T qz{ quat.z };
        return math::Vector3<T>{
            { T(2) * (qx * qz - qw * qy) },
            { T(2) * (qw * qx + qy * qz) },
            { qw * qw - qx * qx - qy * qy + qz * qz }
        };
    }

    template<typename T, AhrsMode M>
    math::Vector3<T> AhrsFilter<T, M>::NorthFromQuaternion(const math::Quaternion<T>& quat, T bx, T bz)
    {
        T qw{ quat.w };
        T qx{ quat.x };
        T qy{ quat.y };
        T qz{ quat.z };
        return math::Vector3<T>{
            { bx * (T(1) - T(2) * (qy * qy + qz * qz)) + bz * T(2) * (qx * qz - qw * qy) },
            { bx * T(2) * (qx * qy - qw * qz) + bz * T(2) * (qw * qx + qy * qz) },
            { bx * T(2) * (qw * qy + qx * qz) + bz * (T(1) - T(2) * (qx * qx + qy * qy)) }
        };
    }

    template<typename T, AhrsMode M>
    math::Quaternion<T> AhrsFilter<T, M>::GradientGravity(const math::Quaternion<T>& quat, const math::Vector3<T>& a)
    {
        T qw{ quat.w };
        T qx{ quat.x };
        T qy{ quat.y };
        T qz{ quat.z };
        T ax{ a.at(0, 0) };
        T ay{ a.at(1, 0) };
        T az{ a.at(2, 0) };

        T f1{ T(2) * (qx * qz - qw * qy) - ax };
        T f2{ T(2) * (qw * qx + qy * qz) - ay };
        T f3{ T(1) - T(2) * (qx * qx + qy * qy) - az };

        T gw{ -T(2) * qy * f1 + T(2) * qx * f2 };
        T gx{ T(2) * qz * f1 + T(2) * qw * f2 - T(4) * qx * f3 };
        T gy{ -T(2) * qw * f1 + T(2) * qz * f2 - T(4) * qy * f3 };
        T gz{ T(2) * qx * f1 + T(2) * qy * f2 };

        T invNorm{ SafeInvSqrt(gw * gw + gx * gx + gy * gy + gz * gz) };
        return math::Quaternion<T>{ gw * invNorm, gx * invNorm, gy * invNorm, gz * invNorm };
    }

    template<typename T, AhrsMode M>
    math::Quaternion<T> AhrsFilter<T, M>::GradientMag(const math::Quaternion<T>& quat, const math::Vector3<T>& m, T bx, T bz)
    {
        T qw{ quat.w };
        T qx{ quat.x };
        T qy{ quat.y };
        T qz{ quat.z };
        T mx{ m.at(0, 0) };
        T my{ m.at(1, 0) };
        T mz{ m.at(2, 0) };

        T f4{ bx * (T(1) - T(2) * (qy * qy + qz * qz)) + bz * T(2) * (qx * qz - qw * qy) - mx };
        T f5{ bx * T(2) * (qx * qy - qw * qz) + bz * T(2) * (qw * qx + qy * qz) - my };
        T f6{ bx * T(2) * (qw * qy + qx * qz) + bz * (T(1) - T(2) * (qx * qx + qy * qy)) - mz };

        T gw{ -T(2) * bz * qy * f4 + (-T(2) * bx * qz + T(2) * bz * qx) * f5 + T(2) * bx * qy * f6 };
        T gx{ T(2) * bz * qz * f4 + (T(2) * bx * qy + T(2) * bz * qw) * f5 + (T(2) * bx * qz - T(2) * bz * qw) * f6 };
        T gy{ (-T(4) * bx * qy - T(2) * bz * qw) * f4 + (T(2) * bx * qx + T(2) * bz * qz) * f5 + (T(2) * bx * qw + T(2) * bz * qy) * f6 };
        T gz{ (-T(4) * bx * qz + T(2) * bz * qx) * f4 + (-T(2) * bx * qw + T(2) * bz * qy) * f5 + T(2) * bx * qx * f6 };

        T invNorm{ SafeInvSqrt(gw * gw + gx * gx + gy * gy + gz * gz) };
        return math::Quaternion<T>{ gw * invNorm, gx * invNorm, gy * invNorm, gz * invNorm };
    }

    template<typename T, AhrsMode M>
    void AhrsFilter<T, M>::IntegrateGyro(const math::Vector3<T>& gyro)
    {
        math::Quaternion<T> qGyro{ T{}, gyro.at(0, 0), gyro.at(1, 0), gyro.at(2, 0) };
        math::Quaternion<T> qDot{ q * qGyro };
        q.w += T(0.5) * qDot.w * Ts;
        q.x += T(0.5) * qDot.x * Ts;
        q.y += T(0.5) * qDot.y * Ts;
        q.z += T(0.5) * qDot.z * Ts;
        q.Normalize();
    }

    template<typename T, AhrsMode M>
    void AhrsFilter<T, M>::MahonyStep(const math::Vector3<T>& gyro, const math::Vector3<T>& e)
    {
        integralFb.at(0, 0) += Ki * e.at(0, 0) * Ts;
        integralFb.at(1, 0) += Ki * e.at(1, 0) * Ts;
        integralFb.at(2, 0) += Ki * e.at(2, 0) * Ts;
        T wx{ gyro.at(0, 0) + Kp * e.at(0, 0) + integralFb.at(0, 0) };
        T wy{ gyro.at(1, 0) + Kp * e.at(1, 0) + integralFb.at(1, 0) };
        T wz{ gyro.at(2, 0) + Kp * e.at(2, 0) + integralFb.at(2, 0) };
        math::Quaternion<T> qGyro{ T{}, wx, wy, wz };
        math::Quaternion<T> qDot{ q * qGyro };
        q.w += T(0.5) * qDot.w * Ts;
        q.x += T(0.5) * qDot.x * Ts;
        q.y += T(0.5) * qDot.y * Ts;
        q.z += T(0.5) * qDot.z * Ts;
    }

    template<typename T, AhrsMode M>
    void AhrsFilter<T, M>::MadgwickStep(const math::Vector3<T>& gyro, const math::Quaternion<T>& grad)
    {
        math::Quaternion<T> qGyro{ T{}, gyro.at(0, 0), gyro.at(1, 0), gyro.at(2, 0) };
        math::Quaternion<T> qDot{ q * qGyro };
        q.w += (T(0.5) * qDot.w - beta * grad.w) * Ts;
        q.x += (T(0.5) * qDot.x - beta * grad.x) * Ts;
        q.y += (T(0.5) * qDot.y - beta * grad.y) * Ts;
        q.z += (T(0.5) * qDot.z - beta * grad.z) * Ts;
    }

    template<typename T, AhrsMode M>
    OPTIMIZE_FOR_SPEED void AhrsFilter<T, M>::UpdateImu(math::Vector3<T> gyro, math::Vector3<T> accel)
    {
        T aNorm{ math::VectorNorm(accel) };
        if (aNorm < std::numeric_limits<T>::epsilon())
        {
            IntegrateGyro(gyro);
            return;
        }

        accel = NormalizeVec(accel, aNorm);

        if constexpr (M == AhrsMode::Mahony)
        {
            math::Vector3<T> e{ math::CrossProduct(accel, GravityFromQuaternion(q)) };
            MahonyStep(gyro, e);
        }
        else
        {
            MadgwickStep(gyro, GradientGravity(q, accel));
        }

        q.Normalize();
    }

    template<typename T, AhrsMode M>
    OPTIMIZE_FOR_SPEED void AhrsFilter<T, M>::UpdateMarg(math::Vector3<T> gyro, math::Vector3<T> accel, math::Vector3<T> mag)
    {
        T aNorm{ math::VectorNorm(accel) };
        if (aNorm < std::numeric_limits<T>::epsilon())
        {
            IntegrateGyro(gyro);
            return;
        }

        T mNorm{ math::VectorNorm(mag) };
        if (mNorm < std::numeric_limits<T>::epsilon())
        {
            UpdateImu(gyro, accel);
            return;
        }

        accel = NormalizeVec(accel, aNorm);
        mag = NormalizeVec(mag, mNorm);

        math::Vector3<T> h{ q.Rotate(mag) };
        T bx{ math::Sqrt(h.at(0, 0) * h.at(0, 0) + h.at(1, 0) * h.at(1, 0)) };
        T bz{ h.at(2, 0) };

        if constexpr (M == AhrsMode::Mahony)
        {
            math::Vector3<T> eAccel{ math::CrossProduct(accel, GravityFromQuaternion(q)) };
            math::Vector3<T> eMag{ math::CrossProduct(mag, NorthFromQuaternion(q, bx, bz)) };
            math::Vector3<T> e{
                { eAccel.at(0, 0) + eMag.at(0, 0) },
                { eAccel.at(1, 0) + eMag.at(1, 0) },
                { eAccel.at(2, 0) + eMag.at(2, 0) }
            };
            MahonyStep(gyro, e);
        }
        else
        {
            math::Quaternion<T> gradG{ GradientGravity(q, accel) };
            math::Quaternion<T> gradM{ GradientMag(q, mag, bx, bz) };
            math::Quaternion<T> grad{
                gradG.w + gradM.w,
                gradG.x + gradM.x,
                gradG.y + gradM.y,
                gradG.z + gradM.z
            };
            T invNorm{ SafeInvSqrt(grad.w * grad.w + grad.x * grad.x + grad.y * grad.y + grad.z * grad.z) };
            grad.w *= invNorm;
            grad.x *= invNorm;
            grad.y *= invNorm;
            grad.z *= invNorm;
            MadgwickStep(gyro, grad);
        }

        q.Normalize();
    }

    template<typename T, AhrsMode M>
    math::Quaternion<T> AhrsFilter<T, M>::Orientation() const
    {
        return q;
    }

    template<typename T, AhrsMode M>
    math::Vector3<T> AhrsFilter<T, M>::Euler() const
    {
        return q.ToEulerZYX();
    }

    template<typename T, AhrsMode M>
    void AhrsFilter<T, M>::Reset()
    {
        q = math::Quaternion<T>::Identity();
        integralFb = math::Vector3<T>{ { T{} }, { T{} }, { T{} } };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class AhrsFilter<float, AhrsMode::Madgwick>;
    extern template class AhrsFilter<float, AhrsMode::Mahony>;
#endif
}
