#pragma once

#include "infra/util/Function.hpp"
#include <chrono>

namespace controllers
{
    template<typename QNumberType>
    class PidDriver
    {
    public:
        virtual void Read(const infra::Function<void(QNumberType)>& onDone) = 0;
        virtual void ControlAction(QNumberType) = 0;
        virtual void Start(std::chrono::system_clock::duration sampleTime) = 0;
        virtual void Stop() = 0;
    };
}
