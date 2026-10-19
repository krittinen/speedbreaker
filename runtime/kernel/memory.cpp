// Adapted from Unleashed Recompiled (https://github.com/hedge-dev/UnleashedRecomp),
// GPL-3.0-or-later. Modified for SpeedBreaker.
#include "../stdafx.h"
#include "memory.h"

#ifdef _WIN32
bool Memory::MapPhysicalMirrors()
{
    constexpr size_t physicalSize = 0x20000000;
    HANDLE mapping = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0,
        DWORD(physicalSize), nullptr);
    if (!mapping)
    {
        failedError = int(GetLastError());
        return false;
    }

    void* views[3]{};
    constexpr uint32_t offsets[] = { 0xA0000000, 0xC0000000, 0xE0000000 };
    for (size_t i = 0; i < std::size(views); i++)
    {
        views[i] = MapViewOfFileEx(mapping, FILE_MAP_ALL_ACCESS, 0, 0, physicalSize, base + offsets[i]);
        if (views[i] != base + offsets[i])
        {
            failedError = int(GetLastError());
            for (void* view : views)
                if (view)
                    UnmapViewOfFile(view);
            CloseHandle(mapping);
            return false;
        }
    }
    CloseHandle(mapping);

    // Windows mapping offsets have 64 KiB allocation granularity, so the
    // Xbox's 4 KiB physical offset in the 0xE window cannot be represented.
    eWindowShift = 0;
    return true;
}
#else
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach/mach.h>
#endif

bool Memory::MapPhysicalMirrors()
{
    constexpr size_t PHYSICAL_SIZE = 0x20000000;  // 512 MB
#ifdef __APPLE__
    // A named memory entry, mapped three times with vm_map. The iOS sandbox
    // refuses shm_open (EPERM); this works there and on macOS alike.
    memory_object_size_t entrySize = PHYSICAL_SIZE;
    mach_port_t entry = MACH_PORT_NULL;
    kern_return_t kr = mach_make_memory_entry_64(mach_task_self(), &entrySize, 0,
        MAP_MEM_NAMED_CREATE | VM_PROT_READ | VM_PROT_WRITE, &entry, MACH_PORT_NULL);
    if (kr != KERN_SUCCESS)
    {
        fprintf(stderr, "[memory] mach_make_memory_entry_64: %s\n", mach_error_string(kr));
        failedError = kr;
        return false;
    }
    bool ok = true;
#else
    char name[64];
    snprintf(name, sizeof(name), "/speedbreaker-%d", int(getpid()));
    int fd = shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0600);
    if (fd < 0)
    {
        failedError = errno;
        perror("[memory] shm_open");
        return false;
    }
    shm_unlink(name);  // the mappings keep it alive
    bool ok = ftruncate(fd, PHYSICAL_SIZE) == 0;
    if (!ok)
        failedError = errno;
#endif

    struct View { uint32_t guest; size_t offset; size_t length; };
    const View views[] = {
        { 0xA0000000, 0, PHYSICAL_SIZE },
        { 0xC0000000, 0, PHYSICAL_SIZE },
        { 0xE0000000, 0x1000, PHYSICAL_SIZE - 0x1000 },
    };
    for (const View& v : views)
    {
        if (!ok)
            break;
        void* at = base + v.guest;
        // mmap offsets must be host-page aligned. On 16 KB-page hosts (Apple
        // Silicon) the 0xE0000000 window's 4 KB shift can't be expressed, so
        // it is mapped unshifted there; MmGetPhysicalAddress uses
        // eWindowShift so addresses stay consistent. 4 KB-page hosts (x86
        // Linux, the Steam Deck) get the exact hardware layout.
        size_t offset = v.offset;
        if (offset % size_t(sysconf(_SC_PAGESIZE)) != 0)
            offset = 0;
        if (v.guest == 0xE0000000)
            eWindowShift = uint32_t(offset);
#ifdef __APPLE__
        vm_address_t address = vm_address_t(at);
        kr = vm_map(mach_task_self(), &address, v.length, 0, VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE, entry,
            vm_offset_t(offset), FALSE, VM_PROT_READ | VM_PROT_WRITE, VM_PROT_READ | VM_PROT_WRITE, VM_INHERIT_NONE);
        if (kr != KERN_SUCCESS)
        {
            fprintf(stderr, "[memory] vm_map physical view: %s\n", mach_error_string(kr));
            failedError = kr;
            ok = false;
        }
#else
        if (mmap(at, v.length, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED, fd, off_t(offset)) == MAP_FAILED)
        {
            failedError = errno;
            perror("[memory] mmap physical view");
            ok = false;
        }
#endif
    }
#ifdef __APPLE__
    mach_port_deallocate(mach_task_self(), entry);  // the mappings keep it alive
#else
    close(fd);
#endif
    return ok;
}
#endif

Memory::Memory()
{
#ifdef _WIN32
    const char* testFail = std::getenv("NFSMW_TEST_RESERVE_FAIL");
    const int failStep = testFail ? std::atoi(testFail) : 0;
    if (failStep == 1)
    {
        failedStep = 1;
        failedError = ERROR_NOT_ENOUGH_MEMORY;
        return;
    }

    // Reserve the complete guest address range to choose a base, then replace
    // its physical-memory windows with views of one shared section.
    base = static_cast<uint8_t*>(VirtualAlloc(nullptr, PPC_MEMORY_SIZE, MEM_RESERVE, PAGE_NOACCESS));
    if (!base)
    {
        failedStep = 1;
        failedError = int(GetLastError());
        return;
    }
    uint8_t* candidate = base;
    if (!VirtualFree(candidate, 0, MEM_RELEASE))
    {
        failedStep = 1;
        failedError = int(GetLastError());
        base = nullptr;
        return;
    }

    constexpr size_t nonPhysicalSize = 0xA0000000;
    base = static_cast<uint8_t*>(VirtualAlloc(candidate, nonPhysicalSize,
        MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    if (base != candidate)
    {
        failedStep = 1;
        failedError = int(GetLastError());
        if (base)
            VirtualFree(base, 0, MEM_RELEASE);
        base = nullptr;
        return;
    }
    DWORD oldProtect = 0;
    if (!VirtualProtect(base, 4096, PAGE_NOACCESS, &oldProtect))
    {
        failedStep = 1;
        failedError = int(GetLastError());
        VirtualFree(base, 0, MEM_RELEASE);
        base = nullptr;
        return;
    }

    if (failStep == 2 || !MapPhysicalMirrors())
    {
        failedStep = 2;
        if (failStep == 2)
            failedError = ERROR_NOT_ENOUGH_MEMORY;
        VirtualFree(base, 0, MEM_RELEASE);
        base = nullptr;
        return;
    }
#else
    // NFSMW_TEST_RESERVE_FAIL=1|2: fail the reservation or the mirrors as an
    // iOS without room would (memory_status.h), to see the startup message
    // anywhere.
    const char* testFail = std::getenv("NFSMW_TEST_RESERVE_FAIL");
    int failStep = testFail ? std::atoi(testFail) : 0;

    // SpeedBreaker: one contiguous 4 GB, since guest addresses are offsets
    // from base. iOS makes room for it with Extended Virtual Addressing or
    // Increased Memory Limit (scripts/ios/SpeedBreaker/SpeedBreaker.entitlements,
    // kernel/memory_status.h); without them only about 5 GB is free in one
    // piece, so this runs in static initialization, before anything else
    // takes address space. If it fails (ENOMEM), main.cpp says so on screen.
    base = failStep == 1 ? (uint8_t*)MAP_FAILED
        : (uint8_t*)mmap((void*)0x100000000ull, PPC_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);

    if (base == (uint8_t*)MAP_FAILED && failStep != 1)
        base = (uint8_t*)mmap(NULL, PPC_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);

    if (base == (uint8_t*)MAP_FAILED)
    {
        failedStep = 1;
        failedError = failStep == 1 ? ENOMEM : errno;
        base = nullptr;
        return;
    }

    mprotect(base, 4096, PROT_NONE);

    // SpeedBreaker: the 360 exposes its 512 MB of physical memory through three
    // windows that alias each other: 0xA0000000 and 0xC0000000 map physical 0,
    // 0xE0000000 maps physical +0x1000 (Xenia's map_info). D3D writes records
    // through one window and reads them through another, so all three must be
    // the same host memory: one shared object, mapped three times.
    if (failStep == 2 || !MapPhysicalMirrors())
    {
        failedStep = 2;
        if (failStep == 2)
        {
#ifdef __APPLE__
            failedError = KERN_NO_SPACE;
#else
            failedError = ENOMEM;
#endif
        }
        munmap(base, PPC_MEMORY_SIZE);
        base = nullptr;
        return;
    }
#endif

    for (size_t i = 0; PPCFuncMappings[i].guest != 0; i++)
    {
        if (PPCFuncMappings[i].host != nullptr)
            InsertFunction(PPCFuncMappings[i].guest, PPCFuncMappings[i].host);
    }
}

void* MmGetHostAddress(uint32_t ptr)
{
    return g_memory.Translate(ptr);
}
