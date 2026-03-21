#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/Matrix.hpp"
#include <array>

namespace dynamics
{
    template<typename T>
    struct RevoluteJointLink
    {
        T mass;
        math::SquareMatrix<T, 3> inertia; // inertia tensor at center of mass, in link frame
        math::Vector<T, 3> jointAxis;     // joint rotation axis in link frame (unit vector)
        math::Vector<T, 3> parentToJoint; // position of this joint origin in parent frame
        math::Vector<T, 3> jointToCoM;    // position of center of mass in link frame
    };
}
