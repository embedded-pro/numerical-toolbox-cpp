#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/dynamics/RevoluteJointLink.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>

namespace kinematics
{
    template<typename T, std::size_t NumLinks>
    class ForwardKinematics
    {
        static_assert(std::is_floating_point_v<T>,
            "ForwardKinematics only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<dynamics::RevoluteJointLink<T>, NumLinks>;
        using PositionArray = std::array<Vector3, NumLinks + 1>;

        ForwardKinematics() = default;

        // Computes the 3D positions of all joints (including base at index 0
        // and end-effector at index NumLinks) given link descriptions and
        // joint angles.
        OPTIMIZE_FOR_SPEED PositionArray Compute(const LinkArray& links,
            const JointVector& q) const;
    };

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        typename ForwardKinematics<T, NumLinks>::PositionArray
        ForwardKinematics<T, NumLinks>::Compute(const LinkArray& links,
            const JointVector& q) const
    {
        PositionArray positions{};
        positions[0] = Vector3{};

        auto R = Matrix3::Identity();

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            R = R * math::RotationAboutAxis(links[i].jointAxis, q.at(i, 0));

            Vector3 linkExtent;
            if (i + 1 < NumLinks)
                linkExtent = links[i + 1].parentToJoint;
            else
                linkExtent = links[i].jointToCoM * T(2);

            positions[i + 1] = positions[i] + R * linkExtent;
        }

        return positions;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ForwardKinematics<float, 1>;
    extern template class ForwardKinematics<float, 2>;
    extern template class ForwardKinematics<float, 3>;
#endif
}
