// Copyright (c) 2011-2020 Idiap Research Institute
//
// vfoa_Exceptions - exceptions thrown by the bayes filter module
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)

#ifndef __BF_EXCEPTIONS_H__
#define __BF_EXCEPTIONS_H__

// SYSTEM INCLUDES
#include <string>           // STL string
#include <stdexcept>        // STL exception

namespace Bayes_Filter {

/// Base exception for the bayes filter module
class bf_Exception: public std::runtime_error {
    public:
        bf_Exception(const std::string& iMessage = "") :
            runtime_error(iMessage) {}
};

} // namespace Bayes_Filter

#endif // __BF_EXCEPTIONS_H__
