#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/dynamics/RevoluteJointLink.hpp"
#include "numerical/kinematics/ForwardKinematics.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <array>
#include <cmath>
#include <cstddef>

namespace kinematics
{
    template<typename T>
    struct InverseKinematicsConfig
    {
        static_assert(std::is_floating_point_v<T>,
            "InverseKinematicsConfig only supports floating-point types");

        T dampingFactor = T(0.1);
        T tolerance = T(1e-4);
        std::size_t maxIterations = 100;
    };

    template<typename T, std::size_t NumLinks>
    struct InverseKinematicsResult
    {
        math::Vector<T, NumLinks> q;
        T finalError;
        std::size_t iterations;
        bool converged;
    };

    template<typename T, std::size_t NumLinks>
    class InverseKinematics
    {
        static_assert(std::is_floating_point_v<T>,
            "InverseKinematics only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector3<T>;
        using Matrix3 = math::Matrix3<T>;
        using JointVector = math::Vector<T, NumLinks>;
        using Jacobian = math::Matrix<T, 3, NumLinks>;
        using LinkArray = std::array<dynamics::RevoluteJointLink<T>, NumLinks>;
        using PositionArray = std::array<Vector3, NumLinks + 1>;

        explicit InverseKinematics(InverseKinematicsConfig<T> cfg = {});

        OPTIMIZE_FOR_SPEED InverseKinematicsResult<T, NumLinks> Solve(
            const LinkArray& links,
            const Vector3& target,
            const JointVector& initialQ) const;

    private:
        Jacobian ComputeJacobian(
            const LinkArray& links,
            const JointVector& q,
            const PositionArray& positions) const;

        InverseKinematicsConfig<T> config;
        ForwardKinematics<T, NumLinks> fk;
        mutable solvers::GaussianElimination<T, 3> solver;
    };

    template<typename T, std::size_t NumLinks>
    InverseKinematics<T, NumLinks>::InverseKinematics(InverseKinematicsConfig<T> cfg)
        : config(cfg)
    {
        really_assert(config.dampingFactor > T(0));
        really_assert(config.tolerance > T(0));
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        InverseKinematicsResult<T, NumLinks>
        InverseKinematics<T, NumLinks>::Solve(
            const LinkArray& links,
            const Vector3& target,
            const JointVector& initialQ) const
    {
        JointVector q = initialQ;

        std::size_t iter = 0;
        T error = T(0);

        for (; iter < config.maxIterations; ++iter)
        {
            auto positions = fk.Compute(links, q);
            const Vector3& eePos = positions[NumLinks];

            Vector3 e = target - eePos;
            error = math::VectorNorm(e);

            if (error < config.tolerance)
                return { q, error, iter, true };

            Jacobian J = ComputeJacobian(links, q, positions);

            math::SquareMatrix<T, 3> JJt{};
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                {
                    T sum = T(0);
                    for (std::size_t k = 0; k < NumLinks; ++k)
                        sum += J.at(r, k) * J.at(c, k);
                    JJt.at(r, c) = sum;
                }

            T lambda2 = config.dampingFactor * config.dampingFactor;
            JJt.at(0, 0) += lambda2;
            JJt.at(1, 1) += lambda2;
            JJt.at(2, 2) += lambda2;

            Vector3 y = solver.Solve(JJt, e);

            for (std::size_t i = 0; i < NumLinks; ++i)
            {
                T dq = T(0);
                for (std::size_t r = 0; r < 3; ++r)
                    dq += J.at(r, i) * y.at(r, 0);
                q.at(i, 0) += dq;
            }
        }

        auto positions = fk.Compute(links, q);
        Vector3 e = target - positions[NumLinks];
        error = math::VectorNorm(e);

        return { q, error, iter, false };
    }

    template<typename T, std::size_t NumLinks>
    typename InverseKinematics<T, NumLinks>::Jacobian
    InverseKinematics<T, NumLinks>::ComputeJacobian(
        const LinkArray& links,
        const JointVector& q,
        const PositionArray& positions) const
    {
        const Vector3& eePos = positions[NumLinks];

        Matrix3 R = Matrix3::Identity();
        Jacobian J{};

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            Vector3 worldAxis = R * links[i].jointAxis;
            Vector3 jointToEe = eePos - positions[i];
            Vector3 col = math::CrossProduct(worldAxis, jointToEe);

            J.at(0, i) = col.at(0, 0);
            J.at(1, i) = col.at(1, 0);
            J.at(2, i) = col.at(2, 0);

            R = R * math::RotationAboutAxis(links[i].jointAxis, q.at(i, 0));
        }

        return J;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class InverseKinematics<float, 1>;
    extern template class InverseKinematics<float, 2>;
    extern template class InverseKinematics<float, 3>;
#endif
}
