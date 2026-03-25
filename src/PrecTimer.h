//# PrecTimer.h: Precision timer to measure elapsed times in a cumulative way
//# Ported to Windows (VS2022) using QueryPerformanceCounter

#ifndef CASA_PRECTIMER_H
#define CASA_PRECTIMER_H

#include <cstdlib>
#include <iostream>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace casa {

  class PrecTimer {
  public:
    PrecTimer();
    ~PrecTimer();

    void start();
    void stop();
    void reset();

    void show() const;
    void show(std::ostream& os) const;
    void show(const std::string&) const;
    void show(std::ostream& os, const std::string& prefix) const;

    double getReal() const;
    unsigned long long getCount() const;

  private:
    void print_time(std::ostream&, double time) const;

#ifdef _WIN32
    LARGE_INTEGER startTime;
    LARGE_INTEGER accumulated;
    static LARGE_INTEGER frequency;
    static bool frequencyInitialized;
#else
    union {
      long long total_time;
      struct {
#if defined __PPC__
        int total_time_high, total_time_low;
#else
        int total_time_low, total_time_high;
#endif
      };
    };
    static double CPU_speed_in_MHz;
    static double get_CPU_speed_in_MHz();
#endif

    unsigned long long count;
  };

  inline void PrecTimer::reset()
  {
#ifdef _WIN32
    accumulated.QuadPart = 0;
#else
    total_time = 0;
#endif
    count = 0;
  }

  inline unsigned long long PrecTimer::getCount() const
  {
    return count;
  }

  inline PrecTimer::PrecTimer()
  {
#ifdef _WIN32
    startTime.QuadPart = 0;
    if (!frequencyInitialized) {
      QueryPerformanceFrequency(&frequency);
      frequencyInitialized = true;
    }
#endif
    reset();
  }

  inline PrecTimer::~PrecTimer()
  {}

  inline void PrecTimer::start()
  {
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    startTime = now;
#elif (defined __i386__ || defined __x86_64__) && (defined __GNUC__ || defined __INTEL_COMPILER)
    asm volatile
    (
      "rdtsc\n\t"
      "subl %%eax, %0\n\t"
      "sbbl %%edx, %1"
    :
      "+m" (total_time_low), "+m" (total_time_high)
    :
    :
      "eax", "edx"
    );
#endif
  }

  inline void PrecTimer::stop()
  {
#ifdef _WIN32
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    accumulated.QuadPart += (now.QuadPart - startTime.QuadPart);
#elif (defined __i386__ || defined __x86_64__) && (defined __GNUC__ || defined __INTEL_COMPILER)
    asm volatile
    (
      "rdtsc\n\t"
      "addl %%eax, %0\n\t"
      "adcl %%edx, %1"
    :
      "+m" (total_time_low), "+m" (total_time_high)
    :
    :
      "eax", "edx"
    );
#endif
    ++count;
  }

} // namespace casa

#endif
