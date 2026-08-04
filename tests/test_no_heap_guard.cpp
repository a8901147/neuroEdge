#ifdef EDGENEURO_HEAP_GUARD_ENABLED

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include "edgeneuro/no_heap_guard.hpp"

using edgeneuro::NoHeapGuard;

TEST_CASE("NoHeapGuard counts allocations while unarmed", "[no_heap_guard]") {
    NoHeapGuard::reset_count();
    REQUIRE(NoHeapGuard::count() == 0);

    {
        std::vector<int> v; // guard is not armed here: allocation is counted, not fatal
        v.resize(64);
        REQUIRE(NoHeapGuard::count() > 0);
    }
}

TEST_CASE("NoHeapGuard stays at zero across an allocation-free armed scope", "[no_heap_guard]") {
    NoHeapGuard::reset_count();
    {
        NoHeapGuard guard;
        std::array<int, 64> stack_only{}; // no heap involved
        volatile int sink = stack_only[0];
        (void)sink;
    }
    REQUIRE(NoHeapGuard::count() == 0);
}

// Proves the guard is not just a counter but genuinely fatal: a real heap
// allocation inside an armed scope must terminate the process via abort().
// Run in a forked child so the parent test process survives to report the
// result — this is a Host-only Catch2 test (POSIX fork), never compiled
// for the STM32 target.
TEST_CASE("NoHeapGuard aborts the process on allocation while armed", "[no_heap_guard]") {
    const pid_t pid = fork();
    REQUIRE(pid >= 0);

    if (pid == 0) {
        // Catch2 installs its own SIGABRT handler in the parent process (to
        // turn crashes inside other test cases into clean failure reports)
        // and fork() inherits that handler into the child. Reset it to the
        // default disposition so the abort() below actually terminates the
        // child via SIGABRT instead of being swallowed by Catch2's handler.
        std::signal(SIGABRT, SIG_DFL);

        // This line is expected to abort() and never return.
        NoHeapGuard guard;
        auto* leak = new int[16];
        // In an optimized build, a `new[]` whose result is otherwise unused
        // is eligible for allocation elision (LLVM will drop the call
        // entirely, defeating this very test). This inline-asm compiler
        // barrier forces `leak` to escape, guaranteeing the call actually
        // executes and reaches our overridden operator new[].
        asm volatile("" : : "r"(leak) : "memory");
        _exit(42); // only reached if the guard failed to catch the allocation
    }

    int status = 0;
    REQUIRE(waitpid(pid, &status, 0) == pid);
    REQUIRE(WIFSIGNALED(status));
    REQUIRE(WTERMSIG(status) == SIGABRT);
}

#endif // EDGENEURO_HEAP_GUARD_ENABLED
