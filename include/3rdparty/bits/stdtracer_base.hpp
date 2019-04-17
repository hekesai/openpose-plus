#pragma once
#include <chrono>

template <typename T, typename clock_t>
std::chrono::duration<T> since(const std::chrono::time_point<clock_t> &t0)
{
    return clock_t::now() - t0;
}
