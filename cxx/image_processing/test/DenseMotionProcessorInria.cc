/**
 * @file cxx/image_processing/test/DenseMotionProcessorInria.cc
 * @date 11 December 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Unit tests for ip_DenseMotionProcessorInria class
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE DenseMotionProcessorInriaTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <fstream>
#include <list>

// PROJECT INCLUDES
#include <image_processing/ip_Exceptions.h>
#include <image_processing/ip_RoiWindow.h>
#include <image_processing/ip_SingleImageProvider.h>
#include <image_processing/ip_GrayscaleImageProvider.h>
#include <image_processing/ip_Dense2DMotionProcessorInria.h>

using namespace ImageProcessing;
using namespace std;

/////////////////////////////// FIXTURE //////////////////////////////////////

namespace ImageProcessing {
    struct ip_Logger {
        ip_Logger()   {
            const char* report_file_path = getenv("TTRACK_TESTREPORT_FILE");
              if (!report_file_path) {
                  std::cerr <<
                  "Environment variable $TTRACK_TESTREPORT_FILE is not set."
                  " Have you setup your working environment correctly?" <<
                  std::endl;
                  throw ip_Exception();
              }
              m_LogStream.open(report_file_path);
              boost::unit_test::unit_test_log.set_stream(m_LogStream);
        }
        ~ip_Logger()  {
            boost::unit_test::unit_test_log.set_stream( std::cout );
            m_LogStream << "</TestLog>";
            m_LogStream.close();
        }

        private:
        std::ofstream m_LogStream;
    };
}

BOOST_GLOBAL_FIXTURE(ip_Logger)

////////////////////////// LOCAL DECLARATIONS ////////////////////////////////

class ip_MultipleImageProvider : public ip_ImageProvider {

    public:
    ip_MultipleImageProvider(const list<IplImage*>& images) :
        mCurrentImageIterator(images.begin()),
        mLastImageIterator(images.end()){
        IplImage * buffer_image = cvCreateImage(
                cvSize((*mCurrentImageIterator)->width,
                       (*mCurrentImageIterator)->height),
                (*mCurrentImageIterator)->depth,
                (*mCurrentImageIterator)->nChannels);
        image(buffer_image);
    }

    virtual ~ip_MultipleImageProvider() {}

    virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_IMAGE_DATA; };

    virtual float fps() const { return 25; }

    protected:

    virtual void recompute_image(IplImage* buffer_image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time) {
        if (mCurrentImageIterator == mLastImageIterator) {
            throw ip_Exception("Last image reached!");
        } else {
            cvCopy(*mCurrentImageIterator++, buffer_image);
        }
    }

    private:
    list<IplImage*>::const_iterator mCurrentImageIterator;
    const list<IplImage*>::const_iterator mLastImageIterator;
};


/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_motion_estimation )

BOOST_AUTO_TEST_CASE( test_motion_estimation_internal_dxdy_1 ) {
    const int width = 10;
    const int height = 10;

    IplImage * image1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage * image2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    cvZero(image1);
    cvZero(image2);

    cvRectangle(image1, cvPoint(2, 2), cvPoint(7, 7), CV_RGB(255, 255, 255),
            CV_FILLED);
    cvRectangle(image2, cvPoint(4, 4), cvPoint(9, 9), CV_RGB(255, 255, 255),
            CV_FILLED);

    list<IplImage*> images;
    images.push_back(image1);
    images.push_back(image2);

    ip_ImageProvider * data_provider = new ip_MultipleImageProvider(images);
    ip_Dense2DMotionProcessorInria * motion_proc =
            new ip_Dense2DMotionProcessorInria(data_provider,
                    ip_Dense2DMotionProcessorInriaConfig::getDefault());

    // process first image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    // process second image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    ip_RoiWindow roi = ip_RoiWindow::from_CvRect(cvRect(0, 0, width, height));
    motion_proc->reestimate4roi(roi);
    CMotion2DModel * model = motion_proc->getModel();

    double parameters[12];
    model->getParameters(parameters);
    BOOST_CHECK_SMALL(fabs(parameters[0] - 2.0), 0.07);
    BOOST_CHECK_SMALL(fabs(parameters[1] - 2.0), 0.07);
    BOOST_CHECK_SMALL(fabs(parameters[2] - 0.0), 0.03);

    delete motion_proc;
    delete data_provider;

    cvReleaseImage(&image2);
    cvReleaseImage(&image1);
}

BOOST_AUTO_TEST_CASE( test_motion_estimation_internal_dxdy_2 ) {
    const int width = 10;
    const int height = 10;

    IplImage * image1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage * image2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    cvZero(image1);
    cvZero(image2);

    cvRectangle(image1, cvPoint(1, 1), cvPoint(6, 6), CV_RGB(255, 255, 255),
            CV_FILLED);
    cvRectangle(image2, cvPoint(4, 4), cvPoint(9, 9), CV_RGB(255, 255, 255),
            CV_FILLED);

    list<IplImage*> images;
    images.push_back(image1);
    images.push_back(image2);

    ip_ImageProvider * data_provider = new ip_MultipleImageProvider(images);
    ip_Dense2DMotionProcessorInria * motion_proc =
            new ip_Dense2DMotionProcessorInria(data_provider,
                    ip_Dense2DMotionProcessorInriaConfig::getDefault());

    // process first image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    // process second image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    ip_RoiWindow roi = ip_RoiWindow::from_CvRect(cvRect(0, 0, width, height));
    motion_proc->reestimate4roi(roi);
    CMotion2DModel * model = motion_proc->getModel();

    double parameters[12];
    model->getParameters(parameters);
    BOOST_CHECK_SMALL(fabs(parameters[0] - 3.0), 0.34);
    BOOST_CHECK_SMALL(fabs(parameters[1] - 3.0), 0.34);
    BOOST_CHECK_SMALL(fabs(parameters[2]), 0.13);

    delete motion_proc;
    delete data_provider;

    cvReleaseImage(&image2);
    cvReleaseImage(&image1);
}

BOOST_AUTO_TEST_CASE( test_motion_estimation_internal_dxdy_3 ) {
    const int width = 10;
    const int height = 10;

    IplImage * image1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage * image2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    cvZero(image1);
    cvZero(image2);

    cvRectangle(image1, cvPoint(1, 1), cvPoint(6, 6), CV_RGB(255, 255, 255),
            CV_FILLED);
    cvRectangle(image2, cvPoint(2, 2), cvPoint(7, 7), CV_RGB(255, 255, 255),
            CV_FILLED);

    list<IplImage*> images;
    images.push_back(image1);
    images.push_back(image2);

    ip_ImageProvider * data_provider = new ip_MultipleImageProvider(images);
    ip_Dense2DMotionProcessorInria * motion_proc =
            new ip_Dense2DMotionProcessorInria(data_provider,
                    ip_Dense2DMotionProcessorInriaConfig::getDefault());

    // process first image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    // process second image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    ip_RoiWindow roi = ip_RoiWindow::from_CvRect(cvRect(0, 0, width, height));
    motion_proc->reestimate4roi(roi);
    CMotion2DModel * model = motion_proc->getModel();

    double parameters[12];
    model->getParameters(parameters);
    BOOST_CHECK_SMALL(fabs(parameters[0] - 1.0), 0.05);
    BOOST_CHECK_SMALL(fabs(parameters[1] - 1.0), 0.05);
    BOOST_CHECK_SMALL(fabs(parameters[2]), 0.02);

    delete motion_proc;
    delete data_provider;

    cvReleaseImage(&image2);
    cvReleaseImage(&image1);
}

BOOST_AUTO_TEST_CASE( test_motion_estimation_internal_div_1 ) {
    const int width = 10;
    const int height = 10;

    IplImage * image1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage * image2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    cvZero(image1);
    cvZero(image2);

    cvRectangle(image1, cvPoint(3, 3), cvPoint(6, 6), CV_RGB(255, 255, 255),
            CV_FILLED);
    cvRectangle(image2, cvPoint(2, 2), cvPoint(7, 7), CV_RGB(255, 255, 255),
            CV_FILLED);

    list<IplImage*> images;
    images.push_back(image1);
    images.push_back(image2);

    ip_ImageProvider * data_provider = new ip_MultipleImageProvider(images);
    ip_Dense2DMotionProcessorInria * motion_proc =
            new ip_Dense2DMotionProcessorInria(data_provider,
                    ip_Dense2DMotionProcessorInriaConfig::getDefault());

    // process first image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    // process second image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    ip_RoiWindow roi = ip_RoiWindow::from_CvRect(cvRect(0, 0, width, height));
    motion_proc->reestimate4roi(roi);
    CMotion2DModel * model = motion_proc->getModel();

    double parameters[12];
    model->getParameters(parameters);
    BOOST_CHECK_SMALL(fabs(parameters[0]), 0.24);
    BOOST_CHECK_SMALL(fabs(parameters[1]), 0.24);
    BOOST_CHECK_SMALL(fabs(parameters[2] - 0.5), 0.04);

    delete motion_proc;
    delete data_provider;

    cvReleaseImage(&image2);
    cvReleaseImage(&image1);
}

BOOST_AUTO_TEST_CASE( test_motion_estimation_internal_div_2 ) {
    const int width = 10;
    const int height = 10;

    IplImage * image1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage * image2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    cvZero(image1);
    cvZero(image2);

    cvRectangle(image1, cvPoint(3, 3), cvPoint(6, 6), CV_RGB(255, 255, 255),
            CV_FILLED);
    cvRectangle(image2, cvPoint(1, 1), cvPoint(8, 8), CV_RGB(255, 255, 255),
            CV_FILLED);

    list<IplImage*> images;
    images.push_back(image1);
    images.push_back(image2);

    ip_ImageProvider * data_provider = new ip_MultipleImageProvider(images);
    ip_Dense2DMotionProcessorInria * motion_proc =
            new ip_Dense2DMotionProcessorInria(data_provider,
                    ip_Dense2DMotionProcessorInriaConfig::getDefault());

    // process first image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    // process second image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    ip_RoiWindow roi = ip_RoiWindow::from_CvRect(cvRect(0, 0, width, height));
    motion_proc->reestimate4roi(roi);
    CMotion2DModel * model = motion_proc->getModel();

    double parameters[12];
    model->getParameters(parameters);
    BOOST_CHECK_SMALL(fabs(parameters[0]), 0.05);
    BOOST_CHECK_SMALL(fabs(parameters[1]), 0.05);
    BOOST_CHECK_SMALL(fabs(parameters[2] - 0.5), 0.02);

    delete motion_proc;
    delete data_provider;

    cvReleaseImage(&image2);
    cvReleaseImage(&image1);
}

BOOST_AUTO_TEST_CASE( test_motion_estimation_external_dxdy ) {
    const int width = 10;
    const int height = 10;
    const float factor = 0.01;
    const float precision = 1e-5;

    IplImage * image1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage * image2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    cvZero(image1);
    cvZero(image2);

    cvRectangle(image1, cvPoint(2, 2), cvPoint(7, 7), CV_RGB(255, 255, 255),
            CV_FILLED);
    cvRectangle(image2, cvPoint(4, 4), cvPoint(9, 9), CV_RGB(255, 255, 255),
            CV_FILLED);

    list<IplImage*> images;
    images.push_back(image1);
    images.push_back(image2);

    ip_ImageProvider * data_provider = new ip_MultipleImageProvider(images);
    ip_Dense2DMotionProcessorInria * motion_proc =
            new ip_Dense2DMotionProcessorInria(data_provider,
                    ip_Dense2DMotionProcessorInriaConfig::getDefault());

    // process first image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    // process second image
    data_provider->invalidate();
    motion_proc->invalidate();
    data_provider->image();
    motion_proc->image();

    CMotion2DModel * model = new CMotion2DModel();
    CMotion2DEstimator * estimator = new CMotion2DEstimator();

    ip_Dense2DMotionProcessorInria::apply_config_options(
            motion_proc->getConfig(), model, estimator);

    ip_RoiWindow roi = ip_RoiWindow::from_CvRect(cvRect(0, 0, width, height));
    motion_proc->reestimate4roi(roi, model, estimator);

    double parameters[12];
    model->getParameters(parameters);
    BOOST_CHECK_CLOSE(parameters[0], 2.0f, 5);
    BOOST_CHECK_CLOSE(parameters[1], 2.0f, 5);

    delete estimator;
    delete model;
    delete motion_proc;
    delete data_provider;

    cvReleaseImage(&image2);
    cvReleaseImage(&image1);
}

BOOST_AUTO_TEST_SUITE_END()
