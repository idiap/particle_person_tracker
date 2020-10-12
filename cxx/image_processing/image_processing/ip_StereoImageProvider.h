/**
 * @file cxx/image_processing/image_processing/ip_StereoImageProvider.h
 * @date 21 January 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Image provider for stereo camera data
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_STEREOIMAGEPROVIDER_H__
#define __IP_STEREOIMAGEPROVIDER_H__

// SYSTEM INCLUDES
#include <string>                    // STL string
#include <highgui.h>                 // for CvCapture

// LOCAL INCLUDES
#include "ip_ImageProvider.h"        // base class

namespace ImageProcessing {

/// @brief Class to represent an image provider for stereo device.
///
/// This provider acts as a regular video file provider,
/// giving access to the left image of a stereo device.
/// The corresponding right image is available upon request.
///
/// Two modes are available for streaming the data:
///     offline (all frames are processed one by one,
///              processing time is not taken into account),
///     online  (skips frames according to time elapsed between
///              the two subsequent requests,
///     uses information on frame rate from the video file).
///
/// OpenCV is used to read video files and information on frame rate.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    21.01.2013

class ip_StereoImageProvider : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor to initialize stereo image provider
    /// @param device_name_left Left input video file
    /// @param device_name_right Right input video file
    /// @param width Preferred image width, -1 for the default device width
    /// @param height Preferred image height, -1 for the default device height
    /// @param fps_pref Preferred FPS rate,  -1 for the default one
    /// @param offline_mode Flag indicating the mode (online/offline)
    ///                          in which the provider is launched.
    ///                     Queries every frame in offline mode, skips as much
    ///                     frames as required to match the elapsed processing
    ///                     time in online mode
    ip_StereoImageProvider(const std::string& device_name_left,
            const std::string& device_name_right,
            int width = -1, int height = -1, int fps_pref  = -1,
            bool offline_mode = false);

    /// Destructor, deallocates captures if they were allocated
    virtual ~ip_StereoImageProvider();


    // OPERATIONS

    /// Overrides base class method, returns IP_IMG_PROVIDER_STEREO
    /// @return Provider's ID: IP_IMG_PROVIDER_STEREO
    /// @see ip_ImageProviderType
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_STEREO;
    };

    /// Overrides base class method.
    /// Returns video files FPS (frames per second) rate
    /// @return Video files FPS (frames per second) rate.
    virtual float fps() const;

    /// Overrides base class method.
    /// Returns current image ID. The ID is equal to image number in the video
    /// file (image 1-based index) in both online and offline modes.
    /// @return Current image ID.
    virtual unsigned image_id() const;

    /// Read/write access to the left image
    /// @return Left image
    IplImage * left_image() {
        return m_LeftImage;
    }

    /// Read/write access to the left image
    /// @return Left image
    IplImage * right_image() {
        return m_RightImage;
    }

    /// Read-only access to the left image
    /// @return Left image
    const IplImage * left_image() const {
        return m_LeftImage;
    }

    /// Read-only access to the right image
    /// @return Right image
    const IplImage * right_image() const {
        return m_RightImage;
    }

    protected:

    // OPERATIONS

    /// Overrides base class method.
    /// Obtains the most recent image from the video file.
    /// Prepares either the next frame (offline mode) or uses
    /// the delay between current time and the time when a frame was
    /// last requested to skip the corresponding number of frames (online mode)
    /// @param image Image to write results to
    /// @param roi   Region of interest for which to recompute the image,
    ///              currently ignored
    /// @param time  Timestamp of the previous image is provided as an input,
    ///              fills current timestamp after processing as an output
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    // Attempts to initialize capture using the device name
    // (path to a video file), throws ip_Exception on error
    // (file does not exist, not a file, video codec not found, etc.)
    CvCapture *  initialize_capture(const std::string& device_name);

    // attempts to make the device stream images of the preferred size
    // returns true if succeeded
    bool set_preferred_image_size();

    CvCapture * m_CaptureLeft;      // OpenCV capture handle
    CvCapture * m_CaptureRight;     // OpenCV capture handle
    IplImage * m_LeftImage;        // right image
    IplImage * m_RightImage;        // right image
    bool m_FirstFrame;              // current frame is the first one?
    const bool m_OfflineMode;       // process every frame if true
    boost::posix_time::ptime m_LastImageTimeStamp;
    int m_PreferredWidth;           // preferred width
    int m_PreferredHeight;          // preferred height
    bool m_Resize;                  // should resize input images?
    float m_FrameDelay_ms;          // time delay between two consecutive frames
    unsigned m_ImageId;
    float m_Fps;                    // video file FPS rate

};

} // namespace ImageProcessing

#endif // __IP_STEREOIMAGEPROVIDER_H__
