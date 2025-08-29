#pragma once

#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename QNumberType>
    class TrajectoryGenerator
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "TrajectoryGenerator can only be instantiated with math::QNumber or floating point types.");

    public:
        struct MotionProfile
        {
            QNumberType position;
            QNumberType velocity;
            QNumberType acceleration;
        };

        virtual void SetTarget(QNumberType targetPosition) = 0;
        virtual MotionProfile Update(QNumberType dt) = 0;
        virtual bool IsComplete() const = 0;
        virtual void Reset(QNumberType currentPosition) = 0;
        virtual void SetInitialConditions(QNumberType position, QNumberType velocity) = 0;
    };
}
