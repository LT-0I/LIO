#ifndef LIO_LIVOX_CPU_AFFINITY_H
#define LIO_LIVOX_CPU_AFFINITY_H

#include <thread>
#include <vector>
#include <sched.h>
#include <pthread.h>

namespace cpu_affinity {

// RK3588 CPU layout:
// CPU 0-3: Cortex-A55 (小核) @ 1.8GHz
// CPU 4-7: Cortex-A76 (大核) @ 2.4GHz

constexpr int RK3588_BIG_CORE_START = 4;
constexpr int RK3588_BIG_CORE_END = 7;
constexpr int RK3588_BIG_CORE_COUNT = 4;

// Bind current thread to big cores (CPU 4-7)
inline bool bindToBigCores() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int i = RK3588_BIG_CORE_START; i <= RK3588_BIG_CORE_END; ++i) {
        CPU_SET(i, &cpuset);
    }
    return pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0;
}

// Bind a std::thread to big cores
inline bool bindToBigCores(std::thread& t) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int i = RK3588_BIG_CORE_START; i <= RK3588_BIG_CORE_END; ++i) {
        CPU_SET(i, &cpuset);
    }
    return pthread_setaffinity_np(t.native_handle(), sizeof(cpuset), &cpuset) == 0;
}

// Bind a pthread to big cores
inline bool bindToBigCores(pthread_t tid) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int i = RK3588_BIG_CORE_START; i <= RK3588_BIG_CORE_END; ++i) {
        CPU_SET(i, &cpuset);
    }
    return pthread_setaffinity_np(tid, sizeof(cpuset), &cpuset) == 0;
}

// Bind to a specific big core (0-3 maps to CPU 4-7)
inline bool bindToBigCore(int core_index) {
    if (core_index < 0 || core_index >= RK3588_BIG_CORE_COUNT) {
        return false;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(RK3588_BIG_CORE_START + core_index, &cpuset);
    return pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0;
}

// Bind a std::thread to a specific big core
inline bool bindToBigCore(std::thread& t, int core_index) {
    if (core_index < 0 || core_index >= RK3588_BIG_CORE_COUNT) {
        return false;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(RK3588_BIG_CORE_START + core_index, &cpuset);
    return pthread_setaffinity_np(t.native_handle(), sizeof(cpuset), &cpuset) == 0;
}

// Get the number of big cores available
inline int getBigCoreCount() {
    return RK3588_BIG_CORE_COUNT;
}

// Helper: create and bind worker threads to big cores
template<typename Func>
std::vector<std::thread> createBigCoreWorkers(int num_workers, Func&& func) {
    std::vector<std::thread> workers;
    int actual_workers = std::min(num_workers, RK3588_BIG_CORE_COUNT);
    workers.reserve(actual_workers);

    for (int i = 0; i < actual_workers; ++i) {
        workers.emplace_back(std::forward<Func>(func), i);
        bindToBigCore(workers.back(), i);
    }
    return workers;
}

} // namespace cpu_affinity

#endif // LIO_LIVOX_CPU_AFFINITY_H
