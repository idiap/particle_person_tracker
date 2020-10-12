/**
 * @file cxx/examples/facetrack/ModelDumperFactory.cc
 * @date 29 January 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Creates model dumper by output source name (file, RSB, etc)
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <boost/filesystem.hpp>                 // file system utils
#include <sstream>                              // string stream

// LOCAL INCLUDES
#include "ModelDumperFactory.h"
#include "ModelDumper_TA2Format.h"
#include "ModelDumper_MOTFormat.h"
#include "ModelDumper_CSVFormat.h"
#ifdef __ROSINTEGRATION_FOUND__
#include "ModelDumper_ROSFormat.h"
#include "ModelDumper_ROSFormat2.h"
#endif


using namespace std;
namespace fs = boost::filesystem;

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

static const string RSB_PROTOCOL_STR = "rsb";
static const string ROS_PROTOCOL_STR = "ros";
static const string TA2_PROTOCOL_STR = "ta2";
static const string MOT_PROTOCOL_STR = "mot";
static const string CSV_PROTOCOL_STR = "csv";

/////////////////////////////// PUBLIC ///////////////////////////////////////

/* static */ ModelDumperFactory *
ModelDumperFactory::instance() {
    static ModelDumperFactory inst;
    return &inst;

} // instance

ModelDumper*
ModelDumperFactory::
create_model_dumper(const std::string& source,
                    const MainModel* model)
{

  const string protocol_delimiter("://");

  // check if protocol was explicitly specified
  string::const_iterator prot_i = search(source.begin(), source.end(),
                                         protocol_delimiter.begin(), protocol_delimiter.end());

  if (prot_i != source.end()) {
    // extract protocol specification
    string protocol;
    protocol.reserve(distance(source.begin(), prot_i));
    transform(source.begin(), prot_i, back_inserter(protocol),
              ptr_fun<int,int>(tolower));
    // extract output specification
    advance(prot_i, protocol_delimiter.length());
    string scope = source.substr(distance(source.begin(), prot_i));
    if (scope.length() == 0) {
      scope = "/";
    } else if (scope[0] != '/') {
      scope.insert(0, "/");
    }

    if (protocol == ROS_PROTOCOL_STR) {
#ifdef __ROSINTEGRATION_FOUND__
      return new ModelDumper_ROSFormat2(scope, model);
#else
      ostringstream oss;
      oss << "ROS protocol dumper was not build,"
        " change build configuration";
      throw runtime_error(oss.str());
#endif
    } else if (protocol == TA2_PROTOCOL_STR) {
      return new ModelDumper_TA2Format(scope, model);
    } else if (protocol == MOT_PROTOCOL_STR) {
      return new ModelDumper_MOTFormat(scope, model);
    } else if (protocol == CSV_PROTOCOL_STR) {
      return new ModelDumper_CSVFormat(scope, model);
    } else {
      ostringstream oss;
      oss << "Unknown dump protocol specified: " << protocol;
      throw runtime_error(oss.str());
    }
  } else {
    // use TA2 dumper if no protocol was provided
    return new ModelDumper_TA2Format(source, model);
  }

} // create_image_provider

/////////////////////////////// PRIVATE //////////////////////////////////////

ModelDumperFactory::ModelDumperFactory() {
} // ModelDumperFactory
