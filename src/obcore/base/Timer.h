/*
 *  Open Robotic Vision and Utilities Library
 *  Copyright (c) 2014 TH-Nuernberg - Christian Merkl
 *  Original authors: Stefan May, Christopher Loerken and Dirk Holz
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
#ifndef __TIMER_H__
#define __TIMER_H__

#include "obcore/base/Time.h"

namespace obvious
{

/**
 * @class Timer
 * @brief This utility provides functions for precise timing measurements.
 * @author Christian Merkl, Stefan May, Christopher Loerken and Dirk Holz
 */
class Timer
{
public:
    /**
     * Default constructor. Timer doesn't start to run.
     */
    Timer(void) { }

    /**
     * Starts the internal timer.
     */
    void start(void);

    /**
     * Function resets the timer and returns the elapsed time.
     * @return elapsed time in [s] since construction of timer or last reset call
     */
    double reset(void);

    /**
     * Return the elapsed time since the last reset.
     * @return elapsed time in sec since the last reset.
     */
    double elapsed(void) const;

private:
    Time _start;
};

} // end namespace obvious

#endif
