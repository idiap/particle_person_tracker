// Copyright (c) 2010-2020 Idiap Research Institute
//
// ut_DummyLogger - stub class implementing logging strategy
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include "utils/ut_DummyLogger.h"    // declaration of this

namespace TTrackUtils {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ut_DummyLogger::ut_DummyLogger() {

} // ut_DummyLogger

ut_DummyLogger::~ut_DummyLogger() {
} // ~ut_DummyLogger

template<typename T>
void ut_DummyLogger::log(const std::string& name, const T& value) {
} // log

} // namespace TTrackUtils
