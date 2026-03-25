//# PrecTimer.cpp: Timing facility - Ported to Windows (VS2022)

#include "PrecTimer.h"
#include <iostream>
#include <iomanip>
#include <string>

using namespace std;

namespace casa {

#ifdef _WIN32
LARGE_INTEGER PrecTimer::frequency = {};
bool PrecTimer::frequencyInitialized = false;
#else
double PrecTimer::CPU_speed_in_MHz = PrecTimer::get_CPU_speed_in_MHz();

double PrecTimer::get_CPU_speed_in_MHz()
{
#if defined __linux__ && \
    (defined __i386__ || defined __x86_64__ || defined __ia64__ || defined __PPC__) && \
    (defined __GNUC__ || defined __INTEL_COMPILER || defined __PATHSCALE__ || defined __xlC__)
  ifstream infile("/proc/cpuinfo");
  char buffer[256], *colon;
  while (infile.good()) {
    infile.getline(buffer, 256);
    if (strncmp("cpu MHz", buffer, 7) == 0
        && (colon = strchr(buffer, ':')) != 0) {
      return atof(colon + 2);
    }
  }
  return 0.0;
#else
  return 0.0;
#endif
}
#endif

double PrecTimer::getReal() const
{
#ifdef _WIN32
  if (frequency.QuadPart == 0) return 0.0;
  return static_cast<double>(accumulated.QuadPart) / static_cast<double>(frequency.QuadPart);
#else
  double time = total_time / 1e6;
  if (CPU_speed_in_MHz > 0) {
    time /= CPU_speed_in_MHz;
  }
  return time;
#endif
}

void PrecTimer::show() const
{
  show(cout);
}

void PrecTimer::show(ostream& os) const
{
#ifdef _WIN32
  if (frequency.QuadPart == 0) {
    os << "could not determine timer frequency\n";
  } else if (count > 0) {
    double total = getReal();
    os << "avg = ";
    print_time(os, total / static_cast<double>(count));
    os << ", total = ";
    print_time(os, total);
    os << ", count = " << count << '\n';
  } else {
    os << "not used\n";
  }
#else
  if (CPU_speed_in_MHz == 0) {
    os << "could not determine CPU speed\n";
  } else if (count > 0) {
    double total = static_cast<double>(total_time);
    os << "avg = ";
    print_time(os, total / static_cast<double>(count));
    os << ", total = ";
    print_time(os, total);
    os << ", count = " << count << '\n';
  } else {
    os << "not used\n";
  }
#endif
}

void PrecTimer::show(const string& s) const
{
  show(cout, s);
}

void PrecTimer::show(ostream &os, const string& s) const
{
  show(os);
}

void PrecTimer::print_time(ostream& os, double time) const
{
#ifdef _WIN32
  // time is already in seconds
  static char units[] = { 'n', 'u', 'm', ' ', 'k' };
  time *= 1e9; // convert to nanoseconds
  int i = 0;
  while (time >= 999.5 && i < 4) {
    time /= 1000.0;
    ++i;
  }
  os << setprecision(3) << setw(5) << time << ' ' << units[i] << 's';
#else
  static char units[] = { 'n', 'u', 'm', ' ', 'k' };
  time = 1000.0 * time / CPU_speed_in_MHz;
  int i = 0;
  while (time >= 999.5 && i < 5) {
    time /= 1000.0;
    ++i;
  }
  os << setprecision(3) << setw(5) << time << ' ' << units[i] << 's';
#endif
}

} // namespace casa
