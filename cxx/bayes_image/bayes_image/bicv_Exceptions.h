// Copyright (c) 2011-2020 Idiap Research Institute
//
// bicv_Exceptions - exceptions thrown by the bayes image module
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_EXCEPTIONS_H__
#define __BICV_EXCEPTIONS_H__

// SYSTEM INCLUDES
#include <string>           // STL string
#include <stdexcept>        // STL exception

namespace BICV {

/// Base exception for the bayes image module
class bicv_Exception: public std::runtime_error {
    public:
    bicv_Exception(const std::string& iMessage = "") :
        runtime_error(iMessage) {}
};

} // namespace BICV

#endif // __BICV_EXCEPTIONS_H__
