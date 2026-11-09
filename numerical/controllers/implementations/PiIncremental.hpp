#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include <type_traits>

namespace controllers
{
    template<typename T>
    class PiIncremental
    {
        static_assert(std::is_floating_point_v<T>, "PiIncremental supports floating-point types");

    public:
        void SetTunings(T proportional, T integral)
        {
            errorGain = proportional + integral;
            previousErrorGain = -proportional;
        }

        void Reset()
        {
            output = T{};
            error = T{};
        }

        OPTIMIZE_FOR_SPEED T Propose(T reference, T measured)
        {
            const auto previousError = error;
            error = reference - measured;
            output = output + errorGain * error + previousErrorGain * previousError;
            return output;
        }

        OPTIMIZE_FOR_SPEED void CommitRealized(T applied)
        {
            output = applied;
        }

    private:
        T errorGain{};
        T previousErrorGain{};
        T output{};
        T error{};
    };

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PiIncremental<float>;
#endif
}
