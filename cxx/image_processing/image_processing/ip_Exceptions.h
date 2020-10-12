// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Exceptions - exceptions thrown by the VFOA module
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//          Carl Scheffler (Carl.Scheffler@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_EXCEPTIONS_H__
#define __IP_EXCEPTIONS_H__

// SYSTEM INCLUDES
#include <stdexcept>

namespace ImageProcessing {

  /// Base exception for image processing module
  class ip_Exception: public std::runtime_error {
      public:
          ip_Exception(const std::string& iMessage = "") :
              runtime_error(iMessage) {}
  };

}

#endif // __IP_EXCEPTIONS_H__
