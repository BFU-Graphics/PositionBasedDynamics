/**
 * @author YC XIANG.
 * @date 02/16, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PERFORMANCE_CHECK_H
#define POSITIONBASEDDYNAMICS_PERFORMANCE_CHECK_H

#include <chrono>

namespace HINASIM::UTILS
{
    template<
            class result_t   = std::chrono::milliseconds,
            class clock_t    = std::chrono::steady_clock,
            class duration_t = std::chrono::milliseconds
    >
    auto since(std::chrono::time_point<clock_t, duration_t> const &start)
    {
        return std::chrono::duration_cast<result_t>(clock_t::now() - start);
    }
}

#endif //POSITIONBASEDDYNAMICS_PERFORMANCE_CHECK_H
