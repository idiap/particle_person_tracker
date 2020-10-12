// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_DataImageProvider - image provider using various devices as data source
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/filesystem/operations.hpp> // file system
#include <sstream>                         // string streams
#include <iostream>                        // IO streams

// LOCAL INCLUDES
#include <image_processing/ip_DataImageProvider.h>    // declaration of this
#include <image_processing/ip_Exceptions.h>           // exceptions

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_DataImageProvider::ip_DataImageProvider( const string& device_name,
    int width, int height) : mCvCapture(0),
                             mPreferredWidth(width),
                             mPreferredHeight(height), mResize(true) {

    mCvCapture = initialize_capture(device_name);
    mDate = boost::gregorian::day_clock::local_day();
    mResize = !set_preferred_image_size();
    IplImage * frame = cvQueryFrame(mCvCapture);
    IplImage * buffer_image = cvCreateImage(
            cvSize(mPreferredWidth, mPreferredHeight),
            frame->depth, frame->nChannels);
    image(buffer_image);

}

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void ip_DataImageProvider::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {
    IplImage * frame = cvQueryFrame(mCvCapture);
    double timestamp = cvGetCaptureProperty(mCvCapture, CV_CAP_PROP_POS_MSEC);
    time = boost::posix_time::ptime(mDate, boost::posix_time::milliseconds(
            static_cast<long>(timestamp)));

    if (!frame) {
        throw ip_Exception("No more frames available!");
    }
    try{
    if (mResize) {
        cvResize(frame, buffer_image, CV_INTER_LINEAR);
    } else {
        cvCopy(frame, buffer_image);
    }
    } catch (...) {
        cout << "Exception caught in data provider" << endl << flush;
    }

}

/////////////////////////////// PRIVATE //////////////////////////////////////

CvCapture * ip_DataImageProvider::initialize_capture(
        const std::string& device_name) {

    CvCapture * capture = 0;

    ostringstream oss;

    // determine device type
    if (!capture) {
        try {
            capture = initialize_file_capture(device_name);
        } catch (ip_Exception& e) {
            oss << "File device: " << e.what() << endl;
        }
    }

    if (!capture) {
        try {
            capture = initialize_camera_capture(device_name);
        } catch (ip_Exception& e) {
            oss << "Camera device: " << e.what() << endl;
        }
    }

    if (!capture) {
        throw ip_Exception(oss.str());
    }
    return capture;
} // initialize_capture

CvCapture * ip_DataImageProvider::initialize_file_capture(
        const std::string& device_name) {

    namespace fs = boost::filesystem;

    ostringstream oss;
    CvCapture * capture = cvCaptureFromFile(device_name.c_str());

    if (!capture) {
        fs::path file_path( device_name.c_str() /*, fs::native*/ );
        if (!fs::exists(file_path)) {
            oss << "path " << device_name << " does not exist!";
            throw ip_Exception(oss.str());
        } else if (!fs::is_regular(file_path)) {
            oss << device_name << " is not a regular file!";
            throw ip_Exception(oss.str());
        } else {
            oss << " error opening OpenCV file capture, codec problem?";
            throw ip_Exception(oss.str());
        }
    }
    return capture;
} // initialize_file_capture

CvCapture * ip_DataImageProvider::initialize_camera_capture(
        const std::string& device_name) {
    ostringstream oss;
    CvCapture * capture = cvCaptureFromCAM(CV_CAP_ANY);

    if (!capture) {
        oss << " error opening OpenCV camera capture!";
        throw ip_Exception(oss.str());
    }
    return capture;
} // initialize_camera_capture

bool ip_DataImageProvider::set_preferred_image_size() {

    bool result = true;

    int device_width = static_cast<int>(cvGetCaptureProperty(
            mCvCapture, CV_CAP_PROP_FRAME_WIDTH));
    int device_height = static_cast<int>(cvGetCaptureProperty(
            mCvCapture, CV_CAP_PROP_FRAME_HEIGHT));

    if (mPreferredWidth == -1) {
        mPreferredWidth = device_width;
    }

    if (mPreferredHeight == -1) {
        mPreferredHeight = device_height;
    }

    if (result && (device_width != mPreferredWidth)) {
        result = cvSetCaptureProperty(mCvCapture, CV_CAP_PROP_FRAME_WIDTH,
                mPreferredWidth);
    }

    if (result && (device_height != mPreferredHeight)) {
        result = cvSetCaptureProperty(mCvCapture, CV_CAP_PROP_FRAME_HEIGHT,
                mPreferredHeight);
    }

    return result;
}

} // namespace ImageProcessing
