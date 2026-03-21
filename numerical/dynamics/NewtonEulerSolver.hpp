#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/dynamics/NewtonEulerBody.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

namespace dynamics
{
    template<typename T>
    struct SpatialAcceleration
    {
        math::Vector<T, 3> linear;
        math::Vector<T, 3> angular;
    };

    template<typename T>
    struct SpatialForce
    {
        math::Vector<T, 3> force;
        math::Vector<T, 3> torque;
    };

    template<typename T>
    class NewtonEulerSolver
    {
        static_assert(std::is_floating_point_v<T>,
            "NewtonEulerSolver only supports floating-point types");

    public:
        using Vector3 = math::Vector<T, 3>;
        using InertiaMatrix = math::SquareMatrix<T, 3>;

        NewtonEulerSolver() = default;

        // Forward dynamics (body-frame formulation):
        //   linearAccel  = force / mass - angularVelocity x linearVelocity
        //   angularAccel = I^{-1} (torque - angularVelocity x (I * angularVelocity))
        OPTIMIZE_FOR_SPEED SpatialAcceleration<T> ForwardDynamics(const NewtonEulerBody<T>& body,
            const Vector3& force, const Vector3& torque,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const;

        // Inverse dynamics (body-frame formulation):
        //   force  = mass * (linearAccel + angularVelocity x linearVelocity)
        //   torque = I * angularAccel + angularVelocity x (I * angularVelocity)
        OPTIMIZE_FOR_SPEED SpatialForce<T> InverseDynamics(const NewtonEulerBody<T>& body,
            const Vector3& linearAcceleration, const Vector3& angularAcceleration,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const;

    private:
        static Vector3 CrossProduct(const Vector3& a, const Vector3& b);
    };

    template<typename T>
    typename NewtonEulerSolver<T>::Vector3
    NewtonEulerSolver<T>::CrossProduct(const Vector3& a, const Vector3& b)
    {
        return Vector3{
            a.at(1, 0) * b.at(2, 0) - a.at(2, 0) * b.at(1, 0),
            a.at(2, 0) * b.at(0, 0) - a.at(0, 0) * b.at(2, 0),
            a.at(0, 0) * b.at(1, 0) - a.at(1, 0) * b.at(0, 0)
        };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED
        SpatialAcceleration<T>
        NewtonEulerSolver<T>::ForwardDynamics(const NewtonEulerBody<T>& body,
            const Vector3& force, const Vector3& torque,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const
    {
        auto mass = body.ComputeMass();
        auto I = body.ComputeInertia();

        auto Iw = I * angularVelocity;
        auto gyroscopic = CrossProduct(angularVelocity, Iw);
        auto rhs = torque - gyroscopic;

        solvers::GaussianElimination<T, 3> solver;
        auto angularAccel = solver.Solve(I, rhs);

        auto wCrossV = CrossProduct(angularVelocity, linearVelocity);
        T inverseMass = T(1.0f) / mass;
        auto linearAccel = force * inverseMass - wCrossV;

        return SpatialAcceleration<T>{ linearAccel, angularAccel };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED
        SpatialForce<T>
        NewtonEulerSolver<T>::InverseDynamics(const NewtonEulerBody<T>& body,
            const Vector3& linearAcceleration, const Vector3& angularAcceleration,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const
    {
        auto mass = body.ComputeMass();
        auto I = body.ComputeInertia();

        auto wCrossV = CrossProduct(angularVelocity, linearVelocity);
        auto resultForce = (linearAcceleration + wCrossV) * mass;

        auto Iw = I * angularVelocity;
        auto gyroscopic = CrossProduct(angularVelocity, Iw);
        auto resultTorque = I * angularAcceleration + gyroscopic;

        return SpatialForce<T>{ resultForce, resultTorque };
    }

    extern template class NewtonEulerSolver<float>;
}
