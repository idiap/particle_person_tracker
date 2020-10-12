/**
 * @file cxx/image_processing/src/ip_StereoImageProvider.cc
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

// SYSTEM INCLUDES
#include <boost/filesystem/operations.hpp>   // file system
#include <sstream>                           // string streams
#include <iostream>                          // IO streams

// PROJECT INCLUDES
#include <image_processing/ip_StereoImageProvider.h>  // declaration of this
#include <image_processing/ip_Exceptions.h>           // exceptions

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_StereoImageProvider::ip_StereoImageProvider(
    const string& device_name_left, const string& device_name_right,
    int width /* = -1 */, int height /* = -1 */, int fps_pref /* = -1 */,
    bool offline_mode /* = false */) :
            m_CaptureLeft(0),
            m_CaptureRight(0),
            m_FirstFrame(true),
            m_OfflineMode(offline_mode),
            m_PreferredWidth(width),
            m_PreferredHeight(height),
            m_Resize(true),
            m_ImageId(0){

    m_CaptureLeft = initialize_capture(device_name_left);
    m_CaptureRight = initialize_capture(device_name_right);

    try {
        m_Fps = cvGetCaptureProperty(m_CaptureLeft, CV_CAP_PROP_FPS);
        float fps_right = cvGetCaptureProperty(m_CaptureRight, CV_CAP_PROP_FPS);
        if (m_Fps != fps_right) {
            ostringstream oss;
            oss << "Frame rate of left camera source (" << m_Fps
                << ") is not equal to the frame rate of the right camera source ("
                << fps_right << ")";
            throw ip_Exception(oss.str());
        }
        m_FrameDelay_ms = 1000.0 / m_Fps;
        m_Resize = !set_preferred_image_size();
        m_FirstFrame = true;

        IplImage * frame;
        frame = cvQueryFrame(m_CaptureLeft);
        m_LeftImage = cvCreateImage(cvSize(frame->width, frame->height),
                frame->depth, frame->nChannels);
        frame = cvQueryFrame(m_CaptureRight);
        m_RightImage = cvCreateImage(cvSize(frame->width, frame->height),
                frame->depth, frame->nChannels);
        if ((m_LeftImage->width != m_RightImage->width) ||
            (m_LeftImage->height != m_RightImage->height)) {
            cvReleaseImage(&m_LeftImage);
            cvReleaseImage(&m_RightImage);
            ostringstream oss;
            oss << "Left camera source size ("
                << m_LeftImage->width << ", " << m_LeftImage->height
                << ") is not equal to the right camera source size ("
                << m_RightImage->width << ", " << m_RightImage->height << ")";
            throw ip_Exception(oss.str());
        }
    } catch (...) {
        cvReleaseCapture(&m_CaptureLeft);
        cvReleaseCapture(&m_CaptureRight);
        throw;
    }
    IplImage * buffer_image = cvCreateImage(
            cvSize(m_PreferredWidth, m_PreferredHeight),
            m_LeftImage->depth, m_LeftImage->nChannels);
    image(buffer_image);
} // ip_StereoImageProvider

/* virtual */ ip_StereoImageProvider::~ip_StereoImageProvider() {
    if (m_CaptureLeft) {
        cvReleaseCapture(&m_CaptureLeft);
        m_CaptureLeft = 0;
    }
    if (m_CaptureRight) {
        cvReleaseCapture(&m_CaptureRight);
        m_CaptureRight = 0;
    }
    if (m_LeftImage) {
        cvReleaseImage(&m_LeftImage);
        m_LeftImage = 0;
    }
    if (m_RightImage) {
        cvReleaseImage(&m_RightImage);
        m_RightImage = 0;
    }
} // ~ip_StereoImageProvider

/* virtual */ float
ip_StereoImageProvider::fps() const {
    return m_Fps;
} // fps

/* virtual */ unsigned
ip_StereoImageProvider::image_id() const {
        return m_ImageId;
}

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void ip_StereoImageProvider::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {

    int num_frames_to_skip = 0;

    if (!m_OfflineMode) {
        if (!m_FirstFrame) {
            boost::posix_time::time_duration time_delay = (time - m_LastImageTimeStamp);
            long time_delay_ms = time_delay.total_milliseconds();
            num_frames_to_skip = static_cast<int>(time_delay_ms /
                    m_FrameDelay_ms + 0.5);
        }

        for (int i = 0; i < num_frames_to_skip - 1; ++i) {
            cvQueryFrame(m_CaptureLeft);
            cvQueryFrame(m_CaptureRight);
            m_ImageId++;
        }
    }

//    cout << "next frame, ts=" << time << "; " <<
//            "prev_ts=" << mLastImageTimeStamp << "; " <<
//            "frame_delay=" << mFrameDelay_ms << "; " <<
//            "current time=" << boost::posix_time::microsec_clock::local_time();
//    cout << ", skipped " <<  num_frames_to_skip << " frames" << endl;

    IplImage * frame_left = cvQueryFrame(m_CaptureLeft);
    IplImage * frame_right = cvQueryFrame(m_CaptureRight);
    m_ImageId++;

    if (!frame_left || !frame_right) {
        throw ip_Exception("No more frames available!");
    }

    m_FirstFrame = false;
    m_LastImageTimeStamp = time;

    try{

        cvCopy(frame_left, m_LeftImage);
        cvCopy(frame_right, m_RightImage);

        if (m_Resize) {
            cvResize(frame_left, buffer_image, CV_INTER_LINEAR);
        } else {
            cvCopy(frame_left, buffer_image);
        }
    } catch (...) {
        cout << "Exception caught in stereo data provider!" << endl << flush;
    }

}

/////////////////////////////// PRIVATE //////////////////////////////////////

CvCapture * ip_StereoImageProvider::initialize_capture(
        const std::string& device_name) {

    CvCapture * capture = 0;

    namespace fs = boost::filesystem;

    ostringstream oss;
    capture = cvCaptureFromFile(device_name.c_str());

    if (!capture) {
        fs::path file_path( device_name.c_str() /*, fs::native*/ );
        if (!fs::exists(file_path)) {
            oss << "File device: path " << device_name << " does not exist!";
            throw ip_Exception(oss.str());
        } else if (!fs::is_regular(file_path)) {
            oss << "File device: " << device_name << " is not a regular file!";
            throw ip_Exception(oss.str());
        } else {
            oss << "File device: error opening OpenCV file capture, codec problem?";
            throw ip_Exception(oss.str());
        }
    }
    return capture;

} // initialize_capture

bool ip_StereoImageProvider::set_preferred_image_size() {

    bool result = true;

    int device_width = static_cast<int>(cvGetCaptureProperty(
            m_CaptureLeft, CV_CAP_PROP_FRAME_WIDTH));
    int device_height = static_cast<int>(cvGetCaptureProperty(
            m_CaptureLeft, CV_CAP_PROP_FRAME_HEIGHT));

    if (m_PreferredWidth == -1) {
        if (m_PreferredHeight == -1) {
            m_PreferredWidth = device_width;
            m_PreferredHeight = device_height;
        } else {
            m_PreferredWidth = m_PreferredHeight * device_width / device_height;
        }
    } else if (m_PreferredHeight == -1) {
        m_PreferredHeight = m_PreferredWidth * device_height / device_width;
    }

    if (result && (device_width != m_PreferredWidth)) {
        result = cvSetCaptureProperty(m_CaptureLeft, CV_CAP_PROP_FRAME_WIDTH,
                m_PreferredWidth);
    }

    if (result && (device_height != m_PreferredHeight)) {
        result = cvSetCaptureProperty(m_CaptureRight, CV_CAP_PROP_FRAME_HEIGHT,
                m_PreferredHeight);
    }

    return result;
} // set_preferred_image_size

} // namespace ImageProcessing
