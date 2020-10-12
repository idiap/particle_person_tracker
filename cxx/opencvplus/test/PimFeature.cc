/**
 * @file cxx/opencvplus/test/PimFeature.cc
 * @date 01 June 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief PimFeature unit tests
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PimFeatureTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <opencvplus/cvp_PimFeature.h>

using namespace OpenCvPlus;

/////////////////////////////// FIXTURE //////////////////////////////////////

namespace OpenCvPlus {
    struct cvp_Logger {
        cvp_Logger()   {
            const char* report_file_path = getenv("TTRACK_TESTREPORT_FILE");
              if (!report_file_path) {
                  std::cerr <<
                  "Environment variable $TTRACK_TESTREPORT_FILE is not set."
                  " Have you setup your working environment correctly?" <<
                  std::endl;
                  throw cvp_Exception();
              }
              m_LogStream.open(report_file_path);
              boost::unit_test::unit_test_log.set_stream(m_LogStream);
        }
        ~cvp_Logger()  {
            boost::unit_test::unit_test_log.set_stream( std::cout );
            m_LogStream << "</TestLog>";
            m_LogStream.close();
        }

        private:
        std::ofstream m_LogStream;
    };
}

BOOST_GLOBAL_FIXTURE(cvp_Logger);

/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_pim_feature )

BOOST_AUTO_TEST_CASE( test_constructor_float ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim_feature = new PimFloat(1, 10, 20);
    BOOST_CHECK_EQUAL(pim_feature->channels(), 1);
    BOOST_CHECK_EQUAL(pim_feature->width(), 10);
    BOOST_CHECK_EQUAL(pim_feature->height(), 20);

    {
        CvMat * pim;
        BOOST_CHECK_NO_THROW(pim = pim_feature->map(0));
        BOOST_CHECK_THROW(pim = pim_feature->map(1), cvp_Exception);

        const int step = pim->step / sizeof(PimFloat::value_type) - pim->width;
        PimFloat::value_type * data = reinterpret_cast<PimFloat::value_type *>(
            pim->data.ptr);

        BOOST_CHECK_EQUAL(pim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(pim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 0.0);
            }
            data += step;
        }
    }

    {
        const CvMat * cpim;
        BOOST_CHECK_NO_THROW(cpim = pim_feature->map(0));
        BOOST_CHECK_THROW(cpim = pim_feature->map(1), cvp_Exception);

        const int step = cpim->step / sizeof(PimFloat::value_type) - cpim->width;
        const PimFloat::value_type * data =
            reinterpret_cast<PimFloat::value_type *>(cpim->data.ptr);

        BOOST_CHECK_EQUAL(cpim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(cpim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 0.0);
            }
            data += step;
        }
    }

}

BOOST_AUTO_TEST_CASE( test_constructor_double ) {

    typedef cvp_PimFeature<double> PimDouble;

    PimDouble * pim_feature = new PimDouble(1, 10, 20);
    BOOST_CHECK_EQUAL(pim_feature->channels(), 1);
    BOOST_CHECK_EQUAL(pim_feature->width(), 10);
    BOOST_CHECK_EQUAL(pim_feature->height(), 20);

    {
        CvMat * pim;
        BOOST_CHECK_NO_THROW(pim = pim_feature->map(0));
        BOOST_CHECK_THROW(pim = pim_feature->map(1), cvp_Exception);

        const int step = pim->step / sizeof(PimDouble::value_type) - pim->width;
        PimDouble::value_type * data = reinterpret_cast<PimDouble::value_type *>(
            pim->data.ptr);

        BOOST_CHECK_EQUAL(pim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(pim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 0.0);
            }
            data += step;
        }
    }

    {
        const CvMat * cpim;
        BOOST_CHECK_NO_THROW(cpim = pim_feature->map(0));
        BOOST_CHECK_THROW(cpim = pim_feature->map(1), cvp_Exception);

        const int step = cpim->step / sizeof(PimDouble::value_type) - cpim->width;
        const PimDouble::value_type * data =
            reinterpret_cast<PimDouble::value_type *>(cpim->data.ptr);

        BOOST_CHECK_EQUAL(cpim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(cpim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 0.0);
            }
            data += step;
        }
    }

}

BOOST_AUTO_TEST_CASE( test_copy_constructor ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim_feature = new PimFloat(1, 10, 20);
    PimFloat * pim_feature_copy = new PimFloat(*pim_feature);

    BOOST_CHECK_EQUAL(pim_feature_copy->channels(), 1);
    BOOST_CHECK_EQUAL(pim_feature_copy->width(), 10);
    BOOST_CHECK_EQUAL(pim_feature_copy->height(), 20);

    {
        CvMat * pim;
        BOOST_CHECK_NO_THROW(pim = pim_feature_copy->map(0));
        BOOST_CHECK_THROW(pim = pim_feature_copy->map(1), cvp_Exception);

        const int step = pim->step / sizeof(PimFloat::value_type) - pim->width;
        PimFloat::value_type * data = reinterpret_cast<PimFloat::value_type *>(
            pim->data.ptr);

        BOOST_CHECK_EQUAL(pim->width, pim_feature_copy->width());
        BOOST_CHECK_EQUAL(pim->height, pim_feature_copy->height());

        for (unsigned row = 0; row < pim_feature_copy->height(); ++row) {
            for (unsigned col = 0; col < pim_feature_copy->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 0.0);
            }
            data += step;
        }
    }

    {
        const CvMat * cpim;
        BOOST_CHECK_NO_THROW(cpim = pim_feature_copy->map(0));
        BOOST_CHECK_THROW(cpim = pim_feature_copy->map(1), cvp_Exception);

        const int step = cpim->step / sizeof(PimFloat::value_type) - cpim->width;
        const PimFloat::value_type * data =
            reinterpret_cast<PimFloat::value_type *>(cpim->data.ptr);

        BOOST_CHECK_EQUAL(cpim->width, pim_feature_copy->width());
        BOOST_CHECK_EQUAL(cpim->height, pim_feature_copy->height());

        for (unsigned row = 0; row < pim_feature_copy->height(); ++row) {
            for (unsigned col = 0; col < pim_feature_copy->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 0.0);
            }
            data += step;
        }
    }

}

BOOST_AUTO_TEST_CASE( test_assignment_const ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim_feature = new PimFloat(1, 10, 20);
    *pim_feature = 1;

    BOOST_CHECK_EQUAL(pim_feature->channels(), 1);
    BOOST_CHECK_EQUAL(pim_feature->width(), 10);
    BOOST_CHECK_EQUAL(pim_feature->height(), 20);

    {
        CvMat * pim;
        BOOST_CHECK_NO_THROW(pim = pim_feature->map(0));
        BOOST_CHECK_THROW(pim = pim_feature->map(1), cvp_Exception);

        const int step = pim->step / sizeof(PimFloat::value_type) - pim->width;
        PimFloat::value_type * data = reinterpret_cast<PimFloat::value_type *>(
            pim->data.ptr);

        BOOST_CHECK_EQUAL(pim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(pim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 1.0);
            }
            data += step;
        }
    }

    {
        const CvMat * cpim;
        BOOST_CHECK_NO_THROW(cpim = pim_feature->map(0));
        BOOST_CHECK_THROW(cpim = pim_feature->map(1), cvp_Exception);

        const int step = cpim->step / sizeof(PimFloat::value_type) - cpim->width;
        const PimFloat::value_type * data =
            reinterpret_cast<PimFloat::value_type *>(cpim->data.ptr);

        BOOST_CHECK_EQUAL(cpim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(cpim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 1.0);
            }
            data += step;
        }
    }

}

BOOST_AUTO_TEST_CASE( test_assignment ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim_feature_orig = new PimFloat(1, 10, 20);
    *pim_feature_orig = 2;

    PimFloat * pim_feature = new PimFloat(1, 10, 20);
    *pim_feature = *pim_feature_orig;

    BOOST_CHECK_EQUAL(pim_feature->channels(), 1);
    BOOST_CHECK_EQUAL(pim_feature->width(), 10);
    BOOST_CHECK_EQUAL(pim_feature->height(), 20);

    {
        CvMat * pim;
        BOOST_CHECK_NO_THROW(pim = pim_feature->map(0));
        BOOST_CHECK_THROW(pim = pim_feature->map(1), cvp_Exception);

        const int step = pim->step / sizeof(PimFloat::value_type) - pim->width;
        PimFloat::value_type * data = reinterpret_cast<PimFloat::value_type *>(
            pim->data.ptr);

        BOOST_CHECK_EQUAL(pim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(pim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 2.0);
            }
            data += step;
        }
    }

    {
        const CvMat * cpim;
        BOOST_CHECK_NO_THROW(cpim = pim_feature->map(0));
        BOOST_CHECK_THROW(cpim = pim_feature->map(1), cvp_Exception);

        const int step = cpim->step / sizeof(PimFloat::value_type) - cpim->width;
        const PimFloat::value_type * data =
            reinterpret_cast<PimFloat::value_type *>(cpim->data.ptr);

        BOOST_CHECK_EQUAL(cpim->width, pim_feature->width());
        BOOST_CHECK_EQUAL(cpim->height, pim_feature->height());

        for (unsigned row = 0; row < pim_feature->height(); ++row) {
            for (unsigned col = 0; col < pim_feature->width(); ++col) {
                BOOST_CHECK_EQUAL(*data++, 2.0);
            }
            data += step;
        }
    }

}

BOOST_AUTO_TEST_CASE( test_assignment_ncfail ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim_feature_orig = new PimFloat(1, 10, 20);
    *pim_feature_orig = 2;

    PimFloat * pim_feature = new PimFloat(2, 10, 20);
    BOOST_CHECK_THROW(*pim_feature = *pim_feature_orig, cvp_Exception);

}

BOOST_AUTO_TEST_CASE( test_assignment_whfail ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim_feature_orig = new PimFloat(1, 10, 20);
    *pim_feature_orig = 2;

    PimFloat * pim_feature = new PimFloat(1, 20, 30);
    BOOST_CHECK_THROW(*pim_feature = *pim_feature_orig, cvp_Exception);

}

BOOST_AUTO_TEST_CASE( test_equality1 ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim1 = new PimFloat(1, 10, 20);
    PimFloat * pim2 = new PimFloat(1, 10, 20);

    BOOST_CHECK(*pim1 == *pim2);
}

BOOST_AUTO_TEST_CASE( test_equality2 ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim1 = new PimFloat(1, 10, 20);
    *pim1 = 1;
    PimFloat * pim2 = new PimFloat(1, 10, 20);
    *pim2 = 1;

    BOOST_CHECK(*pim1 == *pim2);
}

BOOST_AUTO_TEST_CASE( test_inequality1 ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim1 = new PimFloat(1, 10, 20);
    *pim1 = 1;
    PimFloat * pim2 = new PimFloat(1, 10, 20);
    *pim2 = 2;

    BOOST_CHECK(*pim1 != *pim2);
}

BOOST_AUTO_TEST_CASE( test_distance_correlation1 ) {

    typedef cvp_PimFeature<float> PimFloat;

    PimFloat * pim1 = new PimFloat(1, 10, 20);
    *pim1 = 1;
    PimFloat * pim2 = new PimFloat(1, 10, 20);
    *pim2 = 2;

    PimFloat::value_type d1 = distance_correlation_pixelwise(*pim1, *pim2);
    BOOST_CHECK_EQUAL(d1, 1);
    PimFloat::value_type d2 = distance_correlation_global(*pim1, *pim2);
    BOOST_CHECK_EQUAL(d2, 1);

}

BOOST_AUTO_TEST_CASE( test_distance_correlation2 ) {

    typedef cvp_PimFeature<float> PimFloat;

    CvMat * m;

    const unsigned width = 1;
    const unsigned height = 1;

    PimFloat * pim1 = new PimFloat(2, width, height);
    m = pim1->map(0);
    cvSet(m, cvScalar(1.0));
    m = pim1->map(1);
    cvSet(m, cvScalar(2.0));

    PimFloat * pim2 = new PimFloat(2, width, height);
    m = pim2->map(0);
    cvSet(m, cvScalar(3.0));
    m = pim2->map(1);
    cvSet(m, cvScalar(4.0));

    PimFloat::value_type d1 = distance_correlation_pixelwise(*pim1, *pim2);
    BOOST_CHECK_CLOSE(d1, 11.0 / 5.0 / cvSqrt(5), 1e-5);
    PimFloat::value_type d2 = distance_correlation_global(*pim1, *pim2);
    BOOST_CHECK_CLOSE(d2, 11.0 / 5.0 / cvSqrt(5), 1e-5);

}

BOOST_AUTO_TEST_CASE( test_distance_correlation3 ) {

    typedef cvp_PimFeature<float> PimFloat;

    CvMat * m;

    const unsigned width = 3;
    const unsigned height = 3;

    PimFloat * pim1 = new PimFloat(2, width, height);
    m = pim1->map(0);
    cvSet(m, cvScalar(1.0));
    m = pim1->map(1);
    cvSet(m, cvScalar(2.0));

    PimFloat * pim2 = new PimFloat(2, width, height);
    m = pim2->map(0);
    cvSet(m, cvScalar(3.0));
    m = pim2->map(1);
    cvSet(m, cvScalar(4.0));

    PimFloat::value_type d1 = distance_correlation_pixelwise(*pim1, *pim2);
    BOOST_CHECK_CLOSE(d1, 11.0 / 5.0 / cvSqrt(5), 1e-5);
    PimFloat::value_type d2 = distance_correlation_global(*pim1, *pim2);
    BOOST_CHECK_CLOSE(d2, 11.0 / 5.0 / cvSqrt(5), 1e-5);

}

BOOST_AUTO_TEST_CASE( test_distance_correlation4 ) {

    typedef cvp_PimFeature<float> PimFloat;

    CvMat * m;

    const unsigned width = 2;
    const unsigned height = 2;

    PimFloat * pim1 = new PimFloat(2, width, height);
    m = pim1->map(0);
    cvSetReal2D(m, 0, 0, 1.1);
    cvSetReal2D(m, 0, 1, 1.2);
    cvSetReal2D(m, 1, 0, 1.3);
    cvSetReal2D(m, 1, 1, 1.4);
    m = pim1->map(1);
    cvSetReal2D(m, 0, 0, 1.5);
    cvSetReal2D(m, 0, 1, 1.6);
    cvSetReal2D(m, 1, 0, 1.7);
    cvSetReal2D(m, 1, 1, 1.8);

    PimFloat * pim2 = new PimFloat(2, width, height);
    m = pim2->map(0);
    cvSetReal2D(m, 0, 0, 2.1);
    cvSetReal2D(m, 0, 1, 2.2);
    cvSetReal2D(m, 1, 0, 2.3);
    cvSetReal2D(m, 1, 1, 2.4);
    m = pim2->map(1);
    cvSetReal2D(m, 0, 0, 2.5);
    cvSetReal2D(m, 0, 1, 2.6);
    cvSetReal2D(m, 1, 0, 2.7);
    cvSetReal2D(m, 1, 1, 2.8);

    PimFloat::value_type d1 = distance_correlation_pixelwise(*pim1, *pim2);
    float d1_gt =
    (1.1 * 2.1 + 1.5 * 2.5) / sqrt(1.1 * 1.1 + 1.5 * 1.5) / sqrt(2.1 * 2.1 + 2.5 * 2.5) +
    (1.2 * 2.2 + 1.6 * 2.6) / sqrt(1.2 * 1.2 + 1.6 * 1.6) / sqrt(2.2 * 2.2 + 2.6 * 2.6) +
    (1.3 * 2.3 + 1.7 * 2.7) / sqrt(1.3 * 1.3 + 1.7 * 1.7) / sqrt(2.3 * 2.3 + 2.7 * 2.7) +
    (1.4 * 2.4 + 1.8 * 2.8) / sqrt(1.4 * 1.4 + 1.8 * 1.8) / sqrt(2.4 * 2.4 + 2.8 * 2.8);

    BOOST_CHECK_CLOSE(d1, d1_gt / (width * height), 1e-5);

    PimFloat::value_type d2 = distance_correlation_global(*pim1, *pim2);
    float d2_gt =
    (1.1 * 2.1 + 1.2 * 2.2 + 1.3 * 2.3 + 1.4 * 2.4 + 1.5 * 2.5 + 1.6 * 2.6 + 1.7 * 2.7 + 1.8 * 2.8) /
    sqrt(1.1 * 1.1 + 1.2 * 1.2 + 1.3 * 1.3 + 1.4 * 1.4 + 1.5 * 1.5 + 1.6 * 1.6 + 1.7 * 1.7 + 1.8 * 1.8) /
    sqrt(2.1 * 2.1 + 2.2 * 2.2 + 2.3 * 2.3 + 2.4 * 2.4 + 2.5 * 2.5 + 2.6 * 2.6 + 2.7 * 2.7 + 2.8 * 2.8);

    BOOST_CHECK_CLOSE(d2, d2_gt, 1e-5);

}

BOOST_AUTO_TEST_CASE( test_normalisation ) {

    typedef cvp_PimFeature<float> PimFloat;

    CvMat * m;

    PimFloat * pim1 = new PimFloat(2, 3, 3);
    m = pim1->map(0);
    cvSet(m, cvScalar(1.0));
    m = pim1->map(1);
    cvSet(m, cvScalar(3.0));

    pim1->normalise_channels();

    CvMat * temp = cvCreateMat(pim1->height(), pim1->width(), CV_8U);
    cvCmpS(pim1->map(0), 0.25, temp, CV_CMP_NE);
    BOOST_CHECK(!cvCountNonZero(temp));

    cvCmpS(pim1->map(1), 0.75, temp, CV_CMP_NE);
    BOOST_CHECK(!cvCountNonZero(temp));

    cvReleaseMat(&temp);

}

BOOST_AUTO_TEST_SUITE_END()
