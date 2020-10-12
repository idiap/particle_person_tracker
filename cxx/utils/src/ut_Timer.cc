/**
 * @file cxx/utils/src/ut_Timer.cc
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

// LOCAL INCLUDES
#include <utils/ut_Timer.h>

using namespace std;

namespace TTrackUtils {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ut_Timer::ut_Timer(const boost::posix_time::time_duration& timeout) :
        m_Timeout(timeout) {

} // ut_Timer

ut_Timer::~ut_Timer() {
} // ~ut_Timer

void ut_Timer::operator()() {

    // infinite loop
    for(;;) {
        // emit signal
        m_Signal();
        // sleep
//        cout << "Timer: sleeping " << m_Timeout << endl << flush;
        boost::this_thread::sleep(m_Timeout);
//        cout << "Timer: woke up" << endl << flush;
    }

} // operator()

void ut_Timer::start() {
    m_Thread = boost::thread(&ut_Timer::operator(), this);
} // start

void ut_Timer::join() {
    m_Thread.join();
} // join

} // namespace TTrackUtils
