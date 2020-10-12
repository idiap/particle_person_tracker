// Copyright (c) 2011-2020 Idiap Research Institute
//
// vfoa_Exceptions - exceptions thrown by the VFOA module
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//          Carl Scheffler (Carl.Scheffler@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __VFOA_EXCEPTIONS_H__
#define __VFOA_EXCEPTIONS_H__

// SYSTEM INCLUDES
#include <stdexcept>

namespace VFOA {

  /// Base exception for VFOA
  class vfoa_Exception: public std::runtime_error {
      public:
          vfoa_Exception(const std::string& iMessage = "") :
              runtime_error(iMessage) {}
  };

}

#endif // __VFOA_EXCEPTIONS_H__
