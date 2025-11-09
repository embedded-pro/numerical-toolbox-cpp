#pragma once

#include "numerical/controllers/interfaces/PidDriver.hpp"
#include <gmock/gmock.h>

namespace controllers
{
    template<typename T>
    class MockPidDriver
        : public controllers::PidDriver<T>
    {
    public:
        MOCK_METHOD(void, Read, (const infra::Function<void(T)>&), (override));
        MOCK_METHOD(void, ControlAction, (T), (override));
        MOCK_METHOD(void, Start, (std::chrono::system_clock::duration), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };
}
