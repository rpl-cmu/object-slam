/******************************************************************************
* File:             timer.h
*
* Author:           Akash Sharma
* Created:          05/12/20
* Description:      Timer util class
*****************************************************************************/
#ifndef OSLAM_TIMER_H
#define OSLAM_TIMER_H

#include <chrono>
#include <memory>


namespace oslam {

class Timer {
 public:
  static std::chrono::high_resolution_clock::time_point tic() {
    return std::chrono::high_resolution_clock::now();
  }

  // Returns duration in milliseconds by default.
  // call .count() on returned duration to have number of ticks.
  template <typename T = std::chrono::milliseconds>
  static T toc(const std::chrono::high_resolution_clock::time_point& start) {
    return std::chrono::duration_cast<T>(
        std::chrono::high_resolution_clock::now() - start);
  }
};

// Usage: measure<>::execution(function, arguments)
template <typename T = std::chrono::milliseconds>
struct Measure {
  template <typename F, typename... Args>
  static typename T::rep execution(F&& func, Args&&... args) {
    auto start = std::chrono::steady_clock::now();
    std::forward<decltype(func)>(func)(std::forward<Args>(args)...);
    auto duration =
        std::chrono::duration_cast<T>(std::chrono::steady_clock::now() - start);
    return duration.count();
  }
};

} // namespace oslam
#endif /* ifndef OSLAM_TIMER_H */
