// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_FaceDetectorExecutable - executable that invokes face detector
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/thread/thread.hpp>               // boost thread
#include <boost/thread/xtime.hpp>                // boost xtime
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/foreach.hpp>                     // foreach loop
#include <list>

// LOCAL INCLUDES
#include <image_processing/ip_ImageProviderProxy.h>
#include <image_processing/ip_FaceDetectorExecutable.h> // declaration of this

using namespace std;
using namespace OpenCvPlus;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_FaceDetectorExecutable::
ip_FaceDetectorExecutable(ip_ImageProvider *data_provider,
                          OpenCvPlus::cvp_FaceDetectorGroup * face_detector_group,
                          const boost::program_options::variables_map& vm,
                          double delay_ms):
  m_DataProvider(data_provider),
  m_FaceDetectorGroup(face_detector_group),
  m_Delay_ms(delay_ms)
{
    m_ProxyProvider = new ip_ImageProviderProxy(m_DataProvider);
}


ip_FaceDetectorExecutable::
~ip_FaceDetectorExecutable()
{
  delete m_ProxyProvider;
  m_ProxyProvider = 0;
}


void
ip_FaceDetectorExecutable::
operator()()
{
  // infinite loop
  for(;;) {

    IplImage * image = m_ProxyProvider->image();

    // detect faces
    const list<cvp_FaceDescriptor>& faces =
      m_FaceDetectorGroup->detect_faces(image, m_ProxyProvider->time());

    // std::cout << "Time in operator() " << to_simple_string(m_ProxyProvider->time())
    //           << std::endl;

    m_ProxyProvider->invalidate();

    // emit signal
    m_Signal(faces);

    //        cout << "Face detector: reported detected faces" << endl;
    // sleep
    //        cout << "Face detector: sleeping " << m_Delay_ms << " milliseconds..." << endl << flush;
    boost::this_thread::sleep(boost::posix_time::milliseconds(static_cast<long>(m_Delay_ms)));
    //        cout << "Face detector: woke up" << endl << flush;

    //        boost::posix_time::ptime cur_time = boost::posix_time::microsec_clock::local_time();
    //        cout << "Before: " << cur_time << endl;

    //        cout << m_Delay_ms << " milliseconds" << endl;

    //        cur_time = boost::posix_time::microsec_clock::local_time();
    //        cout << "After: " << cur_time << endl;
  }
}


void ip_FaceDetectorExecutable::start() {
//    cout << "Starting face detector thread... ";
    m_Thread = boost::thread(&ip_FaceDetectorExecutable::operator(), this);
//    cout << "constructed thread object" << endl;
} // start

void ip_FaceDetectorExecutable::join() {
//    cout << "Joining face detector thread..." << flush;
    m_Thread.join();
//    cout << "Done!" << endl << flush;
} // join

} // namespace ImageProcessing
