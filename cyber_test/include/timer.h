#ifndef MOTOR_CONTROL_TIMER_H_
#define MOTOR_CONTROL_TIMER_H_

#include <assert.h>
#include <stdint.h>
#include <time.h>

/*!
 * Timer for measuring time elapsed with clock_monotonic
 */
class Timer {
 public:
  /*!
   * Construct and start timer
   */
  explicit Timer() { Start(); }

  /*!
   * Start the timer
   */
  void Start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

  /*!
   * Get milliseconds elapsed
   */
  double GetMs() { return (double)GetNs() / 1.e6; }

  /*!
   * Get microseconds elapsed
   */
  double getWs() { return (double)GetNs() / 1.e3; }

  /*!
   * Get nanoseconds elapsed
   */
  int64_t GetNs() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) + 1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
   * Get seconds elapsed
   */
  double GetSeconds() { return (double)GetNs() / 1.e9; }

  struct timespec _startTime;
};

#endif  // MOTOR_CONTROL_TIMER_H_
