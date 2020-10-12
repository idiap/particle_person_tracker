// Copyright (c) 2011-2020 Idiap Research Institute
//
// FaceDetectorExecutable - executable that invokes a face detector
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __FACEDETECTOREXECUTABLE_HPP__
#define __FACEDETECTOREXECUTABLE_HPP__

// SYSTEM INCLUDES
#include <string>                                   // STL string
#include <boost/signals2/signal.hpp>                // boost signal
#include <boost/thread/thread.hpp>                  // boost threading

// PROJECT INCLUDES
#include <opencvplus/cvp_FaceDetector.h>            // general face detector
#include <opencvplus/cvp_FaceDetectorFactory.h>     // for face detector type
#include <opencvplus/cvp_FaceDetectorGroup.h>       // face detector group

// LOCAL INCLUDES
#include "ip_ImageProvider.h"                       // image provider


namespace ImageProcessing {

/// @brief Executable that invokes a face detector
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

class ip_FaceDetectorExecutable {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param data_provider source images provider
    /// @param face_detector_group Group of face detectors to use
    /// @param vm options to initialize the face detector
    /// @param delay_ms delay between two consecutive executions of the face detector
    ip_FaceDetectorExecutable(ImageProcessing::ip_ImageProvider * data_provider,
            OpenCvPlus::cvp_FaceDetectorGroup * face_detector_group,
            const boost::program_options::variables_map& vm,
            double delay_ms);

    /// Destructor
    ~ip_FaceDetectorExecutable();

    // OPERATIONS

    /// Functor operator, executes the main loop
    void operator()();

    /// Starts the object as a thread
    void start();

    /// Joins the thread
    void join();

    // TYPES
    typedef void FaceDetectorCallback(
            const std::list<OpenCvPlus::cvp_FaceDescriptor> &);

    // FIELDS
    boost::signals2::signal<FaceDetectorCallback> m_Signal;

    private:

    ImageProcessing::ip_ImageProvider * m_DataProvider;  // data provider
    ImageProcessing::ip_ImageProvider * m_ProxyProvider; // proxy provider
    OpenCvPlus::cvp_FaceDetectorGroup * m_FaceDetectorGroup; // FD group
    double m_Delay_ms;                          // delay, in milliseconds
    boost::thread m_Thread;                     // thread to execute in

};

}

#endif // __IP_FACEDETECTOREXECUTABLE_HPP__
