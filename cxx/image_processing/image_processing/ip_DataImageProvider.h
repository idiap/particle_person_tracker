// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_DataImageProvider - image provider using various devices as data source
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_DATAIMAGEPROVIDER_H__
#define __IP_DATAIMAGEPROVIDER_H__

// SYSTEM INCLUDES
#include <string>                    // STL string
#include <highgui.h>                 // for CvCapture

// LOCAL INCLUDES
#include "ip_ImageProvider.h"        // base class

namespace ImageProcessing {

/// @brief Class to represent an image provider for device data
///
/// This class defines an image provider that uses various devices
/// (files, cameras) as data source.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_DataImageProvider : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor to initialize data image provider
    /// @param device_name Input device name
    /// @param width Preferred image width, -1 for the default device width
    /// @param height Preferred image height, -1 for the default device height
    ip_DataImageProvider(const std::string& device_name,
            int width = -1, int height = -1);

    // OPERATIONS

    /// Overrides base class method, returns IP_IMG_PROVIDER_DATA by default
    /// @return Provider's ID
    /// @see ip_ImageProviderType
    virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_DATA; };

    protected:

    // OPERATIONS

    /// Overrides base class method, obtains the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    // attempts to initialize capture using the device name
    CvCapture *  initialize_capture(const std::string& device_name);

    // attempts to initialize file capture using the device name
    CvCapture *  initialize_file_capture(const std::string& device_name);

    // attempts to initialize camera capture using the device name
    CvCapture *  initialize_camera_capture(const std::string& device_name);

    // attempts to make the device stream images of the preferred size
    // returns true if succeeded
    bool set_preferred_image_size();

    CvCapture * mCvCapture;        // OpenCV capture handle
    boost::gregorian::date mDate; // date to remember capture start
    int mPreferredWidth;           // preferred width
    int mPreferredHeight;          // preferred height
    bool mResize;                  // should resize input images?

};

} // namespace ImageProcessing

#endif // __IP_DATAIMAGEPROVIDER_H__
