#include "FaultHandler.hpp"

#include <cstdio>

extern "C" [[noreturn]] void _exit(int);

namespace
{
    constexpr uint32_t kCfsr     = 0xE000ED28u;
    constexpr uint32_t kHfsr     = 0xE000ED2Cu;
    constexpr uint32_t kMmfar    = 0xE000ED34u;
    constexpr uint32_t kBfar     = 0xE000ED38u;
    constexpr uint32_t kFlashEnd = 0x00400000u;

    uint32_t ScbReg(uint32_t addr)
    {
        return *reinterpret_cast<volatile const uint32_t*>(addr);
    }

    bool IsCodeAddr(uint32_t v)
    {
        return (v & 1u) != 0u && (v & ~1u) < kFlashEnd;
    }
}

extern "C" void FaultDispatch(uint32_t* sp, uint32_t exLr);

extern "C" __attribute__((naked, noreturn)) void HardFault_Handler()
{
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mov r1, lr\n"
        "b FaultDispatch\n"
    );
}

extern "C" __attribute__((naked, noreturn)) void MemManage_Handler()
{
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mov r1, lr\n"
        "b FaultDispatch\n"
    );
}

extern "C" __attribute__((naked, noreturn)) void BusFault_Handler()
{
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mov r1, lr\n"
        "b FaultDispatch\n"
    );
}

extern "C" __attribute__((naked, noreturn)) void UsageFault_Handler()
{
    __asm volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "mov r1, lr\n"
        "b FaultDispatch\n"
    );
}

extern "C" void FaultDispatch(uint32_t* sp, uint32_t exLr)
{
    FaultHandler::Dispatch(sp, exLr);
}

void FaultHandler::Dispatch(uint32_t* sp, uint32_t exLr)
{
    const uint32_t r0      = sp[0];
    const uint32_t r1      = sp[1];
    const uint32_t r2      = sp[2];
    const uint32_t r3      = sp[3];
    const uint32_t r12     = sp[4];
    const uint32_t lrFault = sp[5];
    const uint32_t pc      = sp[6];
    const uint32_t xpsr    = sp[7];

    const uint32_t hfsr  = ScbReg(kHfsr);
    const uint32_t cfsr  = ScbReg(kCfsr);
    const uint32_t mmfar = ScbReg(kMmfar);
    const uint32_t bfar  = ScbReg(kBfar);

    const uint32_t ipsr  = xpsr & 0x1FFu;
    const uint32_t mmfsr = cfsr & 0xFFu;
    const uint32_t bfsr  = (cfsr >> 8) & 0xFFu;
    const uint32_t ufsr  = cfsr >> 16;

    const char* faultName;
    switch (ipsr)
    {
        case 3:  faultName = "HardFault";  break;
        case 4:  faultName = "MemManage";  break;
        case 5:  faultName = "BusFault";   break;
        case 6:  faultName = "UsageFault"; break;
        default: faultName = "Unknown";    break;
    }

    printf("\n\n=== FAULT: %s ===\n", faultName);

    printf("PC   0x%08x  LR   0x%08x  SP   0x%08x\n",
           (unsigned)pc,
           (unsigned)lrFault,
           (unsigned)(uintptr_t)sp);
    printf("xPSR 0x%08x  EXC_RETURN 0x%08x\n",
           (unsigned)xpsr,
           (unsigned)exLr);
    printf("R0   0x%08x  R1   0x%08x  R2   0x%08x\n",
           (unsigned)r0, (unsigned)r1, (unsigned)r2);
    printf("R3   0x%08x  R12  0x%08x\n",
           (unsigned)r3, (unsigned)r12);

    printf("\nHFSR 0x%08x:", (unsigned)hfsr);
    if (hfsr & (1u << 1))  printf(" VECTTBL");
    if (hfsr & (1u << 30)) printf(" FORCED");
    if (hfsr & (1u << 31)) printf(" DEBUGEVT");
    printf("\n");

    printf("CFSR 0x%08x:", (unsigned)cfsr);
    if (mmfsr & (1u << 0)) printf(" IACCVIOL");
    if (mmfsr & (1u << 1)) printf(" DACCVIOL");
    if (mmfsr & (1u << 3)) printf(" MUNSTKERR");
    if (mmfsr & (1u << 4)) printf(" MSTKERR");
    if (mmfsr & (1u << 5)) printf(" MLSPERR");
    if (mmfsr & (1u << 7)) printf(" MMFAR=0x%08x", (unsigned)mmfar);
    if (bfsr  & (1u << 0)) printf(" IBUSERR");
    if (bfsr  & (1u << 1)) printf(" PRECISERR");
    if (bfsr  & (1u << 2)) printf(" IMPRECISERR");
    if (bfsr  & (1u << 3)) printf(" UNSTKERR");
    if (bfsr  & (1u << 4)) printf(" STKERR");
    if (bfsr  & (1u << 5)) printf(" LSPERR");
    if (bfsr  & (1u << 7)) printf(" BFAR=0x%08x", (unsigned)bfar);
    if (ufsr  & (1u << 0)) printf(" UNDEFINSTR");
    if (ufsr  & (1u << 1)) printf(" INVSTATE");
    if (ufsr  & (1u << 2)) printf(" INVPC");
    if (ufsr  & (1u << 3)) printf(" NOCP");
    if (ufsr  & (1u << 4)) printf(" STKOF");
    if (ufsr  & (1u << 8)) printf(" UNALIGNED");
    if (ufsr  & (1u << 9)) printf(" DIVBYZERO");
    printf("\n");

    const bool extendedFrame    = (exLr & (1u << 4)) == 0u;
    const uint32_t* traceStart  = sp + 8u + (extendedFrame ? 18u : 0u);
    const uint32_t* traceEnd    = traceStart + 64u;

    printf("\nStack trace (potential return addresses):\n");
    for (const uint32_t* p = traceStart; p < traceEnd; ++p)
    {
        if (IsCodeAddr(*p))
            printf("  0x%08x\n", (unsigned)(*p & ~1u));
    }

    printf("==================\n\n");
    _exit(1);
}
