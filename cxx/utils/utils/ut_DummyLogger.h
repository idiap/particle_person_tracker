// Copyright (c) 2010-2020 Idiap Research Institute
//
// ut_DummyLogger - stub class implementing logging strategy
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __UT_DUMMYLOGGER_H__
#define __UT_DUMMYLOGGER_H__

// SYSTEM INCLUDES

#include <string>

namespace TTrackUtils {

/// @brief Stub class implementing logging strategy
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    17.08.2011

class ut_DummyLogger {

public:

    /// Constructor
    ut_DummyLogger();

    /// Destructor
    ~ut_DummyLogger();

    /// Template method to log various events
    /// @param name Name of the log entry
    /// @param value A streamable value of some type T
    template<typename T>
    void log(const std::string& name, const T& value);

};

} // namespace TTrackUtils

#endif // __UT_DUMMYLOGGER_H__
