#include "Timer.h"

namespace obvious {

void Timer::start(void)
{
    _start = Time::now();
}

double Timer::reset(void)
{
    Time old(_start);
    _start = Time::now();
    return _start - old;
}

double Timer::elapsed(void) const
{
    return Time::now() - _start;
}

} // end namespace obvious
