#pragma once

#include <cstdint>

class FaultHandler
{
public:
    static void Dispatch(uint32_t* stackFrame, uint32_t exceptionLR) __attribute__((noreturn));
};
