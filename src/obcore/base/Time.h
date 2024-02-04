/*
 *  Open Robotic Vision and Utilities Library
 *  Copyright (c) 2014 TH-Nuernberg - Christian Merkl
 *  E-Mail: christian.merkl@th-nuernberg.de
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __TIME_H__
#define __TIME_H__

#include <ostream>
#include <stdint.h>

namespace obvious {

/**
 * @class Time
 * @brief A simple class to get the current time in sec and usec from the date 1970. You can easly calculate a
 * duration time with the operator-.
 * @author Christian Merkl
 */
class Time
{
public:
    /**
     * Default constructor. Sets all members to 0.
     */
    Time(void);
    /**
     * Copy constructor
     * @param time will be copied.
     */
    Time(const Time& time) : _seconds(time._seconds), _mus(time._mus) { }

    /**
     * Gets the hours of this day.
     * @return hours of this day.
     */
    double hours(void) const;

    /**
     * Gets the time in minutes of this day.
     * @return minutes of this day.
     */
    double min(void) const;

    /**
     * Gets the time in seconds of this day.
     * @return seconds of this day.
     */
    double sec(void) const;

    /**
     * Gets the time in museconds of this day.
     * @return mus of this day.
     */
    double mus(void) const;

     /**
     * Gets the current time.
     * @return the current time as Time object.
     */
    static Time now(void);

    /**
     * Assignment operator.
     * @param time will be copied.
     */
    Time& operator=(const Time& time);

    /**
     * Calculates the time difference between this and time object.
     * @param time will be subtracted from this.
     */
    double operator-(const Time& time) const;

    /**
     * Adds the given sec to this object.
     * @param sec will be added to this object.
     */
    Time& operator+=(const double sec);

    friend std::ostream& operator<<(std::ostream& out, const Time& time);

private:
    uint32_t _seconds;
    uint32_t _mus;

#ifdef WIN32
    static LONG64 _ticksPerSecond;
#endif
};

std::ostream& operator<<(std::ostream& out, const Time& time);

} // end namespace obvious

#endif
