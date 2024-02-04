#include "Time.h"

#ifdef WIN32
#include <windows.h>
#else
#include <cmath>
#include <sys/time.h>
#endif

namespace obvious {

#ifdef WIN32
LONG64 Time::_ticksPerSecond = 0;
#endif

Time::Time(void)
{
#ifdef WIN32
    if(!_ticksPerSecond) QueryPerformanceFrequency((LARGE_INTEGER*)&_ticksPerSecond);
#endif

    _seconds = 0;
    _mus = 0;
}

Time Time::now(void)
{
    Time time;

#ifdef WIN32
    LARGE_INTEGER tick;
    QueryPerformanceCounter(&tick);
    tick.QuadPart = tick.QuadPart * 1000 / _ticksPerSecond;
    time._seconds = tick.u.LowPart;
    time._mus = tick.u.LowPart * 1e3 - time._seconds * 1e6;
#else
    timeval clock;
    ::gettimeofday(&clock, 0);
    time._seconds = clock.tv_sec;
    time._mus = clock.tv_usec;
#endif

    return time;
}

double Time::hours(void) const
{
    return static_cast<double>(_seconds) / 3600.0;
}

double Time::min(void) const
{
    return static_cast<double>(_seconds) / 60.0;
}

double Time::sec(void) const
{
    return static_cast<double>(_seconds) + static_cast<double>(_mus) * 1.0e-6;
}

double Time::mus(void) const
{
    return static_cast<double>(_seconds) * 1.0e6 + static_cast<double>(_mus);
}

Time& Time::operator=(const Time& time)
{
    _seconds = time._seconds;
    _mus = time._mus;
    return *this;
}

double Time::operator-(const Time& time) const
{
    return this->sec() - time.sec();
}

Time& Time::operator+=(const double sec)
{
    _seconds += static_cast<int>(sec);
    _mus += std::fmod(1.0f, sec) * 1.0e-6;
    return *this;
}

std::ostream& operator<<(std::ostream& out, const Time& time)
{
    out << "[" << time._seconds / 3600 << ":"
        << time._seconds % 3600 << ":"
        << time._seconds % 60 << "]";

    return out;
}

} // end namespace obvious
