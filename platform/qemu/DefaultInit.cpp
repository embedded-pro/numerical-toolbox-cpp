#include <cerrno>
#include <cwchar>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

extern char _end;
extern char _heap_end;

extern "C"
{
    caddr_t _sbrk(int incr)
    {
        static char* heap = &_end;
        char* prev = heap;
        incr = (incr + 3) & ~3;
        if (heap + incr > &_heap_end)
        {
            errno = ENOMEM;
            return reinterpret_cast<caddr_t>(-1);
        }
        heap += incr;
        return reinterpret_cast<caddr_t>(prev);
    }

    [[gnu::weak]] void Default_Handler_Forwarded()
    {
        _exit(1);
    }

    void abort()
    {
        _exit(1);
    }

    int _fstat(int, struct stat* st)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }

    void _init()
    {}

    void HardwareInitialization()
    {}

    int _write(int, const char* ptr, int len)
    {
        struct
        {
            int handle;
            const void* ptr;
            unsigned len;
        } block = { 1, ptr, static_cast<unsigned>(len) };

        int result;
        __asm volatile(
            "mov r0, #5\n"
            "mov r1, %[b]\n"
            "bkpt #0xAB\n"
            "mov %[r], r0\n"
            : [r] "=r"(result)
            : [b] "r"(&block)
            : "r0", "r1", "memory");
        return len - result;
    }

    char* getcwd(char* buf, size_t size)
    {
        if (buf && size > 0)
            buf[0] = '\0';
        return buf;
    }

    int mkdir(const char*, mode_t)
    {
        errno = ENOSYS;
        return -1;
    }

    int swprintf(wchar_t*, size_t, const wchar_t*, ...)
    {
        return -1;
    }
}
