// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ImageProviderType - collection of IDs of pre-defined image providers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_IMAGEPROVIDERTYPE_H__
#define __IP_IMAGEPROVIDERTYPE_H__

// SYSTEM INCLUDES
#include <string>

namespace ImageProcessing {

/// @brief A collection of IDs of pre-defined image providers
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

enum ip_ImageProviderType {

    IP_IMG_PROVIDER_GROUP              = 1,  // group of image providers

    IP_IMG_PROVIDER_RED_CHANNEL        = 2,  // red colour channel
    IP_IMG_PROVIDER_GREEN_CHANNEL      = 3,  // green colour channel
    IP_IMG_PROVIDER_BLUE_CHANNEL       = 4,  // blue colour channel
    IP_IMG_PROVIDER_CYAN_CHANNEL       = 5,  // cyan colour channel
    IP_IMG_PROVIDER_MAGENTA_CHANNEL    = 6,  // magenta colour channel
    IP_IMG_PROVIDER_YELLOW_CHANNEL     = 7,  // yellow colour channel
    IP_IMG_PROVIDER_BLACK_CHANNEL      = 8,  // black colour channel
    IP_IMG_PROVIDER_HUE_CHANNEL        = 9,  // hue colour channel
    IP_IMG_PROVIDER_SATURATION_CHANNEL = 10, // saturation colour channel
    IP_IMG_PROVIDER_BRIGHTNESS_CHANNEL = 11, // brightness colour channel
    IP_IMG_PROVIDER_GRAYSCALE          = 12, // grayscale image

    IP_IMG_PROVIDER_GRADIENT_X         = 13, // x-coordinate of gradient
    IP_IMG_PROVIDER_GRADIENT_Y         = 14, // y-coordinate of gradient
    IP_IMG_PROVIDER_GRADIENT_MAGNITUDE = 15, // gradient magnitude
    IP_IMG_PROVIDER_GRADIENT_ANGLE     = 16, // gradient angle

    IP_IMG_PROVIDER_MOTION_X         = 17, // x-coordinate of gradient
    IP_IMG_PROVIDER_MOTION_Y         = 18, // y-coordinate of gradient
    IP_IMG_PROVIDER_MOTION_MAGNITUDE = 19, // gradient magnitude
    IP_IMG_PROVIDER_MOTION_ANGLE     = 20, // gradient angle

    IP_IMG_PROVIDER_SKIN_MASK          = 21, // skin colour
    IP_IMG_PROVIDER_HISTOGRAM          = 22, // histogram data

    IP_IMG_PROVIDER_BG_PROBA           = 23, // background probability

    IP_IMG_PROVIDER_DATA               = 40, // device image provider
    IP_IMG_PROVIDER_FILE_DATA          = 41, // video file image provider
    IP_IMG_PROVIDER_FILE_LIST_DATA     = 42, // video file image provider
    IP_IMG_PROVIDER_WEBCAM_DATA        = 43, // web camera image provider
    IP_IMG_PROVIDER_IMAGE_DATA         = 44, // single image file provider
    IP_IMG_PROVIDER_STEREO             = 45, // stereo video file provider
    IP_IMG_PROVIDER_RSB                = 46, // external RSB provider
    IP_IMG_PROVIDER_STEREO_RSB         = 47, // external RSB stereo provider

    IP_IMG_PROVIDER_DISPARITY          = 48, // disparity and depth provider

    IP_IMG_PROVIDER_ROS                = 49, // external ROS provider

    IP_IMG_PROVIDER_UNKNOWN            = 99  // unknown image provider

};

inline std::string str(ip_ImageProviderType iptype) {
    switch(iptype) {
    case IP_IMG_PROVIDER_WEBCAM_DATA:
        return "webcam";
    case IP_IMG_PROVIDER_FILE_DATA:
        return "video file";
    case IP_IMG_PROVIDER_FILE_LIST_DATA:
        return "image list";
    case IP_IMG_PROVIDER_IMAGE_DATA:
        return "image";
    case IP_IMG_PROVIDER_STEREO_RSB:
        return "RSB stereo";
    case IP_IMG_PROVIDER_RSB:
        return "RSB";
    case IP_IMG_PROVIDER_ROS:
        return "ROS";
    case IP_IMG_PROVIDER_STEREO:
        return "stereo video files";
    default:
        return "unknown";
    }
}

} // namespace ImageProcessing

#endif // __IP_IMAGEPROVIDERTYPE_H__
