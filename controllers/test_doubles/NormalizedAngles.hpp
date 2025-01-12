#ifndef CONTROLLERS_TEST_DOUBLES_NORMALIZED_ANGLES_HPP
#define CONTROLLERS_TEST_DOUBLES_NORMALIZED_ANGLES_HPP

#include "math/QNumber.hpp"

namespace controllers
{
    template<typename T>
    T CreateNormalizedAngle(float angle)
    {
        const float TWO_PI = 2.0f * M_PI;
        const float MAX_VALUE = 0.9999f;

        if (std::abs(angle) < 1e-6f)
            return 0.0f;

        float normalizedTo2Pi = std::fmod(angle, TWO_PI);
        if (normalizedTo2Pi < 0)
            normalizedTo2Pi += TWO_PI;

        return T(normalizedTo2Pi / TWO_PI) * MAX_VALUE;
    }
}

#endif
