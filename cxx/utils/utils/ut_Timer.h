/**
 * @file cxx/utils/utils/ut_Timer.h
 * @date 23 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Timer firing events at fixed rate
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __UT_TIMER_HPP__
#define __UT_TIMER_HPP__

// SYSTEM INCLUDES
#include "boost/date_time/posix_time/posix_time.hpp" // boost time
#include <boost/signals2/signal.hpp>                          // boost signal
#include <boost/thread/thread.hpp>                   // boost threading

namespace TTrackUtils {

/// @brief Timer that fires a signal with certain periodicity.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    26.11.2012

class ut_Timer {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param timeout Timer timeout which defines firing frequency
    ut_Timer(const boost::posix_time::time_duration& timeout);

    /// Destructor
    ~ut_Timer();

    // OPERATIONS

    /// Functor operator, executes the main loop
    void operator()();

    /// Starts the object as a thread
    void start();

    /// Joins the thread
    void join();

    // TYPES
    typedef void TimerCallback();

    // FIELDS
    boost::signals2::signal<TimerCallback> m_Signal;

    private:

    boost::posix_time::time_duration m_Timeout;  // timer delay
    boost::thread m_Thread;                      // thread to execute in

};

} // TTrackUtils

#endif // __UT_TIMER_HPP__
