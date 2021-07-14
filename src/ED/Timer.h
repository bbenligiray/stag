#ifndef TIMER_H
#define TIMER_H

#include <chrono>

namespace edpf {

#define BUILD_MACOS

#ifndef BUILD_MACOS

#include <windows.h>

class Timer {
private:
  __int64 freq, tStart, tStop;

public:
  Timer() {
    // Get the frequency of the hi-res timer
    QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
  } // end-TimerClass

  void Start() {
    // Use hi-res timer
    QueryPerformanceCounter((LARGE_INTEGER *)&tStart);
  } // end-Start

  void Stop() {
    // Perform operations that require timing
    QueryPerformanceCounter((LARGE_INTEGER *)&tStop);
  } // end-Stop

  // Returns time in milliseconds
  double ElapsedTime() {
    // Calculate time difference in milliseconds
    return ((double)(tStop - tStart) / (double)freq) * 1e3;
  } // end-Elapsed
};
#else

class Timer {
private:
  std::chrono::steady_clock::time_point testTime_t1, testTime_t2;

public:
  Timer() {} // end-TimerClass

  void Start() {
    // Use hi-res timer
    testTime_t1 = std::chrono::steady_clock::now();
  } // end-Start

  void Stop() {
    // Perform operations that require timing
    testTime_t2 = std::chrono::steady_clock::now();
  } // end-Stop

  // Returns time in milliseconds
  double ElapsedTime() {
    // Calculate time difference in milliseconds
    return std::chrono::duration_cast<std::chrono::duration<double>>(
               testTime_t2 - testTime_t1)
        .count();
  } // end-Elapsed
};

#endif // !1

} // namespace edpf

#endif