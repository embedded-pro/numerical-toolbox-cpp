#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include <cmath>
#include <numbers>
#include <type_traits>

namespace math
{
    template<typename T>
    class Quaternion
    {
        static_assert(std::is_floating_point_v<T>, "Quaternion supports floating-point types only");

    public:
        Quaternion();
        Quaternion(T w, T x, T y, T z);

        static Quaternion Identity();
        static Quaternion FromAxisAngle(const Vector3<T>& axis, T angle);
        static Quaternion FromRotationMatrix(const Matrix3<T>& r);
        static Quaternion FromEulerZYX(T roll, T pitch, T yaw);
        static Quaternion Slerp(const Quaternion& a, const Quaternion& b, T t);

        OPTIMIZE_FOR_SPEED Quaternion operator*(const Quaternion& rhs) const;
        Quaternion operator-() const;

        Quaternion Conjugate() const;
        Quaternion Inverse() const;
        T Norm() const;
        T SquaredNorm() const;
        Quaternion& Normalize();
        Quaternion Normalized() const;

        OPTIMIZE_FOR_SPEED Vector3<T> Rotate(const Vector3<T>& v) const;

        Matrix3<T> ToRotationMatrix() const;
        Vector3<T> ToEulerZYX() const;

        T w;
        T x;
        T y;
        T z;
    };

    template<typename T>
    Quaternion<T>::Quaternion()
        : w{ T(1) }
        , x{ T(0) }
        , y{ T(0) }
        , z{ T(0) }
    {}

    template<typename T>
    Quaternion<T>::Quaternion(T w, T x, T y, T z)
        : w{ w }
        , x{ x }
        , y{ y }
        , z{ z }
    {}

    template<typename T>
    Quaternion<T> Quaternion<T>::Identity()
    {
        return Quaternion<T>{ T(1), T(0), T(0), T(0) };
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::FromAxisAngle(const Vector3<T>& axis, T angle)
    {
        T halfAngle = angle * T(0.5);
        T s = std::sin(halfAngle);
        T n = VectorNorm(axis);
        T invN = (n > T(0)) ? T(1) / n : T(0);
        return Quaternion<T>{ std::cos(halfAngle), axis.at(0, 0) * invN * s, axis.at(1, 0) * invN * s, axis.at(2, 0) * invN * s };
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::FromRotationMatrix(const Matrix3<T>& r)
    {
        T trace = r.at(0, 0) + r.at(1, 1) + r.at(2, 2);
        Quaternion<T> q;
        if (trace > T(0))
        {
            T s = T(0.5) / std::sqrt(trace + T(1));
            q.w = T(0.25) / s;
            q.x = (r.at(2, 1) - r.at(1, 2)) * s;
            q.y = (r.at(0, 2) - r.at(2, 0)) * s;
            q.z = (r.at(1, 0) - r.at(0, 1)) * s;
        }
        else if (r.at(0, 0) > r.at(1, 1) && r.at(0, 0) > r.at(2, 2))
        {
            T s = T(2) * std::sqrt(T(1) + r.at(0, 0) - r.at(1, 1) - r.at(2, 2));
            q.w = (r.at(2, 1) - r.at(1, 2)) / s;
            q.x = T(0.25) * s;
            q.y = (r.at(0, 1) + r.at(1, 0)) / s;
            q.z = (r.at(0, 2) + r.at(2, 0)) / s;
        }
        else if (r.at(1, 1) > r.at(2, 2))
        {
            T s = T(2) * std::sqrt(T(1) + r.at(1, 1) - r.at(0, 0) - r.at(2, 2));
            q.w = (r.at(0, 2) - r.at(2, 0)) / s;
            q.x = (r.at(0, 1) + r.at(1, 0)) / s;
            q.y = T(0.25) * s;
            q.z = (r.at(1, 2) + r.at(2, 1)) / s;
        }
        else
        {
            T s = T(2) * std::sqrt(T(1) + r.at(2, 2) - r.at(0, 0) - r.at(1, 1));
            q.w = (r.at(1, 0) - r.at(0, 1)) / s;
            q.x = (r.at(0, 2) + r.at(2, 0)) / s;
            q.y = (r.at(1, 2) + r.at(2, 1)) / s;
            q.z = T(0.25) * s;
        }
        return q;
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::FromEulerZYX(T roll, T pitch, T yaw)
    {
        T cr = std::cos(roll * T(0.5));
        T sr = std::sin(roll * T(0.5));
        T cp = std::cos(pitch * T(0.5));
        T sp = std::sin(pitch * T(0.5));
        T cy = std::cos(yaw * T(0.5));
        T sy = std::sin(yaw * T(0.5));
        return Quaternion<T>{
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED Quaternion<T> Quaternion<T>::operator*(const Quaternion& rhs) const
    {
        return Quaternion<T>{
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
        };
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::operator-() const
    {
        return Quaternion<T>{ -w, -x, -y, -z };
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::Conjugate() const
    {
        return Quaternion<T>{ w, -x, -y, -z };
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::Inverse() const
    {
        T sqn = SquaredNorm();
        T invSqn = (sqn > T(0)) ? T(1) / sqn : T(0);
        return Quaternion<T>{ w * invSqn, -x * invSqn, -y * invSqn, -z * invSqn };
    }

    template<typename T>
    T Quaternion<T>::Norm() const
    {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    template<typename T>
    T Quaternion<T>::SquaredNorm() const
    {
        return w * w + x * x + y * y + z * z;
    }

    template<typename T>
    Quaternion<T>& Quaternion<T>::Normalize()
    {
        T n = Norm();
        T invN = (n > T(0)) ? T(1) / n : T(0);
        w *= invN;
        x *= invN;
        y *= invN;
        z *= invN;
        return *this;
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::Normalized() const
    {
        Quaternion<T> copy{ *this };
        copy.Normalize();
        return copy;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED Vector3<T> Quaternion<T>::Rotate(const Vector3<T>& v) const
    {
        Vector3<T> u{ { x }, { y }, { z } };
        Vector3<T> t = CrossProduct(u, v);
        t.at(0, 0) *= T(2);
        t.at(1, 0) *= T(2);
        t.at(2, 0) *= T(2);
        Vector3<T> ct = CrossProduct(u, t);
        return Vector3<T>{
            { v.at(0, 0) + w * t.at(0, 0) + ct.at(0, 0) },
            { v.at(1, 0) + w * t.at(1, 0) + ct.at(1, 0) },
            { v.at(2, 0) + w * t.at(2, 0) + ct.at(2, 0) }
        };
    }

    template<typename T>
    Matrix3<T> Quaternion<T>::ToRotationMatrix() const
    {
        T ww = w * w;
        T xx = x * x;
        T yy = y * y;
        T zz = z * z;
        T wx = w * x;
        T wy = w * y;
        T wz = w * z;
        T xy = x * y;
        T xz = x * z;
        T yz = y * z;
        return Matrix3<T>{
            { ww + xx - yy - zz, T(2) * (xy - wz), T(2) * (xz + wy) },
            { T(2) * (xy + wz), ww - xx + yy - zz, T(2) * (yz - wx) },
            { T(2) * (xz - wy), T(2) * (yz + wx), ww - xx - yy + zz }
        };
    }

    template<typename T>
    Vector3<T> Quaternion<T>::ToEulerZYX() const
    {
        T sinR = T(2) * (w * x + y * z);
        T cosR = T(1) - T(2) * (x * x + y * y);
        T roll = std::atan2(sinR, cosR);

        T sinP = T(2) * (w * y - z * x);
        T pitch;
        if (std::abs(sinP) >= T(1))
            pitch = std::copysign(std::numbers::pi_v<T> / T(2), sinP);
        else
            pitch = std::asin(sinP);

        T sinY = T(2) * (w * z + x * y);
        T cosY = T(1) - T(2) * (y * y + z * z);
        T yaw = std::atan2(sinY, cosY);

        return Vector3<T>{ { roll }, { pitch }, { yaw } };
    }

    template<typename T>
    Quaternion<T> Quaternion<T>::Slerp(const Quaternion& a, const Quaternion& b, T t)
    {
        T d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        Quaternion<T> bAdjusted = b;
        if (d < T(0))
        {
            bAdjusted = -b;
            d = -d;
        }
        if (d > T(0.9995))
        {
            Quaternion<T> result{
                a.w + t * (bAdjusted.w - a.w),
                a.x + t * (bAdjusted.x - a.x),
                a.y + t * (bAdjusted.y - a.y),
                a.z + t * (bAdjusted.z - a.z)
            };
            result.Normalize();
            return result;
        }
        T theta = std::acos(d);
        T sinTheta = std::sin(theta);
        T s0 = std::sin((T(1) - t) * theta) / sinTheta;
        T s1 = std::sin(t * theta) / sinTheta;
        return Quaternion<T>{
            s0 * a.w + s1 * bAdjusted.w,
            s0 * a.x + s1 * bAdjusted.x,
            s0 * a.y + s1 * bAdjusted.y,
            s0 * a.z + s1 * bAdjusted.z
        };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Quaternion<float>;
#endif
}
