// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ImageProviderFactory - image provider facotory to create image providers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#include <boost/filesystem.hpp>

#include <image_processing/ip_ImageProviderFactory.h>
#include <image_processing/ip_Exceptions.h>
#ifdef __ROSINTEGRATION_FOUND__
#include <image_processing/ip_RosImageProvider.h>
#endif


#include "image_processing/ip_WebcamImageProvider.h"
#include "image_processing/ip_FileListImageProvider.h"
#include "image_processing/ip_VideoFileImageProvider.h"
#include "image_processing/ip_StereoImageProvider.h"
#include "image_processing/ip_SingleImageProvider.h"

using namespace std;
namespace fs = boost::filesystem;

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

static const string RSB_PROTOCOL_STR = "rsb";
static const string ROS_PROTOCOL_STR = "ros";

namespace ImageProcessing {

ip_ImageProviderFactory*
ip_ImageProviderFactory::
instance()
{
  static ip_ImageProviderFactory inst;
  return &inst;
}

ip_ImageProvider *
ip_ImageProviderFactory::
create_image_provider(const std::string& source,
                      int width,
                      int height,
                      double scale,
                      int fps,
                      bool offline)
{
  fs::path source_path(source);

  {
    const string protocol_delimiter("://");

    string::const_iterator prot_i = search(source.begin(), source.end(),
                                           protocol_delimiter.begin(),
                                           protocol_delimiter.end());

    if (prot_i != source.end()) {
      // protocol specification provided
      string protocol;
      protocol.reserve(distance(source.begin(), prot_i));
      transform(source.begin(), prot_i, back_inserter(protocol),
                ptr_fun<int,int>(tolower));
      if (protocol == ROS_PROTOCOL_STR) {
#ifdef __ROSINTEGRATION_FOUND__
        advance(prot_i, protocol_delimiter.length());
        string scope = source.substr(distance(source.begin(), prot_i));
        if (scope.length() == 0) {
          scope = "/";
        } else if (scope[0] != '/') {
          scope.insert(0, "/");
        }
        // if (width < 0) {
        //   if (height < 0) {
        //     width = 320;
        //     height = 240;
        //   } else {
        //     width = (4 * height) / 3;
        //   }
        // } else {
        //   if (height < 0) {
        //     height = (width * 3) / 4;
        //   }
        // }
        return new ip_RosImageProvider(scope, width, height, scale);
#else
        throw ip_Exception("ROS image provider was not built,"
                           " change build configuration!");
#endif
      }
    }
  }

  {
    string source_lcase;
    source_lcase.reserve(source.length());
    transform(source.begin(), source.end(), back_inserter(source_lcase),
              ptr_fun<int,int>(tolower));
    if (source_lcase == "webcam") {
      return new ip_WebcamImageProvider(width, height, scale);
    }
  }

  // file list can be provided using %Nd syntax or by a txt file
  if ((source.find('%') != string::npos) ||
      (source_path.extension() == ".txt")) {
    // sequence of files
    return new ip_FileListImageProvider(source,
                                        width, height, scale,
                                        fps, offline);
  }

  {
    size_t pos_colon = source.find(':');
    // stereo image provider
    if ((pos_colon != string::npos)) {
      string file_left = source.substr(0, pos_colon);
      string file_right = source.substr(pos_colon + 1);
      // sequence of files
      return new ip_StereoImageProvider(file_left, file_right,
                                        width, height,
                                        fps, offline);
    }
  }

  if (is_image_file(source)) {
    return new ip_SingleImageProvider(source,
                                      width, height, scale,
                                      fps, offline);
  }

  return new ip_VideoFileImageProvider(source, width, height, scale,
                                       fps, offline);

} // create_image_provider

bool
ip_ImageProviderFactory::
is_image_file(const std::string& filename)
{
  string ext = filename.substr(filename.find_last_of('.') + 1);
  string ext_lcase;
  ext_lcase.reserve(ext.length());
  transform(ext.begin(), ext.end(), back_inserter(ext_lcase),
            ptr_fun<int,int>(tolower));

  switch(ext_lcase.length()) {
  case 2:
    return (ext_lcase == "sr");
  case 3:
    switch (ext_lcase[0]) {
    case 'b':
      return (ext_lcase == "bmp");
    case 'd':
      return (ext_lcase == "dib");
    case 'j':
      return (ext_lcase == "jpg") || (ext_lcase == "jpe");
    case 'p':
      return (ext_lcase == "png") || (ext_lcase == "pbm") ||
        (ext_lcase == "pgm") || (ext_lcase == "ppm");
    case 'r':
      return (ext_lcase == "ras");
    case 't':
      return (ext_lcase == "tif");
    default:
      return false;
    }
    break;
  case 4:
    return (ext_lcase == "jpeg") || (ext_lcase == "tiff");
  default:
    return false;
  }
  return false;
}


ip_ImageProviderFactory::
ip_ImageProviderFactory()
{
}

} // namespace ImageProcessing
