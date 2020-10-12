/**
 * Exceptions for the OpenCVPlus library.
 *
 * @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_EXCEPTIONS_H__
#define __CVP_EXCEPTIONS_H__

#include <stdexcept>

//#include "cvplus.h"

namespace OpenCvPlus {

  /// Base exception for OpenCvPlus
  class cvp_Exception: public std::runtime_error {
  public:
  cvp_Exception(const std::string& iMessage = ""): runtime_error(iMessage) {}
  };

  /// Invalid index exception
  class cvp_ExceptionIndexError: public cvp_Exception {
  public:
  cvp_ExceptionIndexError(const std::string& iMessage = ""): cvp_Exception(iMessage) {}
  };

  /// Invalid value/parameter/argument exception
  class cvp_ExceptionValueError: public cvp_Exception {
  public:
  cvp_ExceptionValueError(const std::string& iMessage = ""): cvp_Exception(iMessage) {}
  };

  /// System input/output error exception
  class cvp_ExceptionIOError: public cvp_Exception {
  public:
  cvp_ExceptionIOError(const std::string& iMessage = ""): cvp_Exception(iMessage) {}
  };

  /// Feature not yet implemented exception
  class cvp_ExceptionNotImplemented: public cvp_Exception {
  public:
  cvp_ExceptionNotImplemented(const std::string& iMessage = ""): cvp_Exception(iMessage) {}
  };

  /// Assertion failed
  class cvp_ExceptionAssertionFailed: public cvp_Exception {
  public:
  cvp_ExceptionAssertionFailed(const std::string& iMessage = ""): cvp_Exception(iMessage) {}
  };

} // namespace OpenCvPlus

#endif
