/**
 * @file cxx/image_processing/src/ip_Dense2DMotionProcessorInria.cc
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Extracts dense 2D motion vector fields from two consecutive images
 * based on CMotion2D software from INRIA.
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

 #ifdef __CMOTION2D_FOUND__

// SYSTEM INCLUDES
#include <sstream>
#include <limits>
#include <iomanip>

// PROJECT INCLUDES
#include <cmotion2d/CMotion2DImage.h>                 // 2D image
#include <cmotion2d/Motion2D.h>                       // for EIdModel
#include <opencvplus/cvp_helper_functions.h>          // for cvp_arctan

// LOCAL INCLUDES
#include <image_processing/ip_PartialImageProvider.h> // for motion components
#include <image_processing/ip_Dense2DMotionProcessorInria.h>

using namespace std;

namespace ImageProcessing {

// enable for the provider to return good values for pixel motion vectors
//#define COMPUTE_MOTION_VECTORS

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

// motion model ID
static const string MOTION_MODEL_ID_INIT = "MDL_AFF_TR_ROT_DIV";
//static const string MOTION_MODEL_ID_INIT = "MDL_AFF_COMPLET";
// flag indicating whether to estimate illumination variation
static const bool MOTION_MODEL_ESTIMATE_ILLUMINATION_VARIATION_INIT = false;

class ip_MotionXProvider : public ip_PartialImageProvider {
    public:
    ip_MotionXProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_MOTION_X;
    };

}; // class ip_MotionXProvider

class ip_MotionYProvider : public ip_PartialImageProvider {
    public:
    ip_MotionYProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_MOTION_Y;
    };

}; // class ip_MotionYProvider

class ip_MotionMagnitudeProvider : public ip_PartialImageProvider {
    public:
    ip_MotionMagnitudeProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_MOTION_MAGNITUDE;
    };

}; // class ip_MotionMagnitudeProvider

class ip_MotionAngleProvider : public ip_PartialImageProvider {
    public:
    ip_MotionAngleProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_MOTION_ANGLE;
    };

}; // class ip_MotionAngleProvider

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_Dense2DMotionProcessorInria::ip_Dense2DMotionProcessorInria(
        ip_ImageProvider * img_provider,
        const ip_Dense2DMotionProcessorInriaConfig& config)
        : ip_Dense2DMotionProcessor(img_provider),
          m_Config(config),
          m_PreviousPyramidBuilt(false) {

    apply_config_options(config, &m_MotionParametricModel, &m_MotionEstimator);

    IplImage* source_grayscale_image = getImageProvider()->image_buffer();
    int height = source_grayscale_image->height;
    int width = source_grayscale_image->width;
    m_Image_8U = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    image(m_Image_8U);

} // ip_Dense2DMotionProcessorInria

/* virtual */
ip_Dense2DMotionProcessorInria::~ip_Dense2DMotionProcessorInria() {
} // ~ip_Dense2DMotionProcessorInria

/* virtual */ void
ip_Dense2DMotionProcessorInria::reestimate4roi(const ip_RoiWindow& roi) {

    ip_Dense2DMotionProcessor::reestimate4roi(roi);

//    cout << "Support ROI: " << roi << endl;

    if (m_PreviousPyramidBuilt) {

        reestimate4roi(roi, &m_MotionParametricModel, &m_MotionEstimator);

#ifdef COMPUTE_MOTION_VECTORS
        double drow, dcol;
        static const double HALF_PI_MUL_100 = 157;

        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                m_MotionParametricModel.getDisplacement(
                        (double) row, (double) col, drow, dcol);
                CV_IMAGE_ELEM(getMotionXProvider()->image_buffer(), float, row, col) = dcol;
                CV_IMAGE_ELEM(getMotionYProvider()->image_buffer(), float, row, col) = drow;
                CV_IMAGE_ELEM(getMotionMagnitudeProvider()->image_buffer(), float, row, col) =
                        cvSqrt(drow * drow + dcol * dcol);
                CV_IMAGE_ELEM(getMotionAngleProvider()->image_buffer(), float, row, col) =
                        cvRound(OpenCvPlus::cvp_arctan(drow / (dcol + FLT_EPSILON))) +
                        HALF_PI_MUL_100;
            }
        }
#endif

//        static int counter = 1;
//        dump_estimator(roi, origin_row, origin_col, S, "estimator/", counter);
//        counter++;
    }
} // reestimate4roi

void
ip_Dense2DMotionProcessorInria::reestimate4roi(
        const ip_RoiWindow& roi, CMotion2DModel * model,
        CMotion2DEstimator * estimator) {

    assert(roi.m_iFirstRow >= 0);
    assert(roi.m_iFirstColumn >= 0);

    if (m_PreviousPyramidBuilt) {
        // obtain the most recent image from the supplier
        IplImage* source_grayscale_image = getImageProvider()->image();

        assert(roi.m_iFirstRow + roi.m_iHeight <= source_grayscale_image->height);
        assert(roi.m_iFirstColumn + roi.m_iWidth <= source_grayscale_image->width);

        // estimate motion
        CMotion2DImage<unsigned char> S;  // Motion estimator support
        const int height = source_grayscale_image->height;
        const int width = source_grayscale_image->width;
        const double origin_row = roi.m_iFirstRow + roi.m_iHeight / 2;
        const double origin_col = roi.m_iFirstColumn + roi.m_iWidth / 2;
        model->setOrigin(origin_row, origin_col);

        const int label = 234;
        S.Init(height, width, 0);

        // KLUDGE: use memset to initialize support
        for (int i = roi.m_iFirstRow; i < roi.m_iFirstRow + roi.m_iHeight; ++i) {
            for (int j = roi.m_iFirstColumn; j < roi.m_iFirstColumn + roi.m_iWidth; ++j) {
                S[i][j] = label;
            }
        }

        // estimate 2D motion from the previous image (first parameter)
        // to the current image (second parameter)
        // pyramid1 - pyramid over the current image
        // pyramid2 - pyramid over the previous image
        // S - support
        /*bool state =*/estimator->estimate(m_Pyramid2, m_Pyramid1,
                S.bitmap, label, *model);
    }
} // reestimate4roi

/* static */ void
ip_Dense2DMotionProcessorInria::apply_config_options(
        const ip_Dense2DMotionProcessorInriaConfig& config,
        CMotion2DModel * model,
        CMotion2DEstimator * estimator) {

    model->reset();
    model->setIdModel(config.m_ModelId);
    model->setVarLight(config.m_EstimateIlluminationVariationFlag);
    bool ierr = estimator->setFirstEstimationLevel(
            config.m_PyramidLevelsNum - 1);
    ierr = estimator->setLastEstimationLevel(1);

    estimator->setRobustEstimator(config.m_RobustEstimationFlag);
    estimator->setRobustFunction(config.m_RobustFunctionType);
    estimator->setCConstant(config.m_CConstantType, config.m_CConstant);
    estimator->setReliableSupportRate(config.m_ReliableSupportRate);
    estimator->setNIterationMaxIRLS(config.m_NumIterIRLS);
    estimator->computeSupportSize(config.m_ComputeSupportSizeFlag);
    estimator->computeCovarianceMatrix(config.m_ComputeCovarianceMatrixFlag);

} // apply_config_options

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void
ip_Dense2DMotionProcessorInria::create_motion_providers(
        ip_ImageProvider * image_provider,
        ip_ImageProvider *& motx_provider, ip_ImageProvider *& moty_provider,
        ip_ImageProvider *& motmagn_provider,
        ip_ImageProvider *& motang_provider) {

    int width = image_provider->image_buffer()->width;
    int height = image_provider->image_buffer()->height;
    int depth = IPL_DEPTH_32F;

    m_MotionParametricModel.setOrigin(width / 2, height / 2);
    m_Pyramid1.allocate(height, width, m_Config.m_PyramidLevelsNum);
    m_Pyramid2.allocate(height, width, m_Config.m_PyramidLevelsNum);

    IplImage * motx_image = cvCreateImage(cvSize(width, height), depth, 1);
    cvZero(motx_image);
    motx_provider = new ip_MotionXProvider(this, motx_image);
    IplImage * moty_image = cvCreateImage(cvSize(width, height), depth, 1);
    cvZero(moty_image);
    moty_provider = new ip_MotionYProvider(this, moty_image);
    IplImage * motmagn_image = cvCreateImage(cvSize(width, height), depth, 1);
    cvZero(motmagn_image);
    motmagn_provider = new ip_MotionMagnitudeProvider(this, motmagn_image);
    IplImage * motang_image = cvCreateImage(cvSize(width, height), depth, 1);
    cvZero(motang_image);
    motang_provider = new ip_MotionAngleProvider(this, motang_image);
//    cout << "Created images and providers " << this << endl;

} // create_motion_providers

/* virtual */ void ip_Dense2DMotionProcessorInria::delete_motion_providers(
    ip_ImageProvider *& motx_provider, ip_ImageProvider *& moty_provider,
    ip_ImageProvider *& motmagn_provider, ip_ImageProvider *& motang_provider) {

    IplImage * motx_image = motx_provider->image_buffer();
    cvReleaseImage(&motx_image);
    IplImage * moty_image = moty_provider->image_buffer();
    cvReleaseImage(&moty_image);
    IplImage * motmagn_image = motmagn_provider->image_buffer();
    cvReleaseImage(&motmagn_image);
    IplImage * motang_image = motang_provider->image_buffer();
    cvReleaseImage(&motang_image);

    m_Pyramid1.destroy();
    m_Pyramid2.destroy();

    ip_Dense2DMotionProcessor::delete_motion_providers(motx_provider,
            moty_provider, motmagn_provider, motang_provider);
//    cout << "Deleted images and providers " << this << endl;

} // delete_motion_providers

/* virtual */ void ip_Dense2DMotionProcessorInria::recompute_image(
        IplImage* image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {

    ip_Dense2DMotionProcessor::recompute_image(image, roi, time);

    if (m_PreviousPyramidBuilt) {
        m_Pyramid1.exchange(m_Pyramid2);
    }

//    cout << "Recompute image ROI: " << roi << endl;

    // obtain the most recent image from the supplier
    IplImage* source_grayscale_image = getImageProvider()->image();
    time = getImageProvider()->time();
    vector<unsigned char> bitmap(source_grayscale_image->width *
            source_grayscale_image->height);

    // KLUDGE: stupid way of copying the data!
    unsigned char * bitmap_ptr = &bitmap[0];

    switch (source_grayscale_image->depth) {
    case IPL_DEPTH_32F:
        cvConvertScale(source_grayscale_image, m_Image_8U,
                numeric_limits<unsigned char>::max());
        for (int row = 0; row < source_grayscale_image->height; ++row) {
            for (int col = 0; col < source_grayscale_image->width; ++col) {
                *bitmap_ptr++ = CV_IMAGE_ELEM(m_Image_8U,
                        unsigned char, row, col);
            }
        }
        break;
    case IPL_DEPTH_8U:
        for (int row = 0; row < source_grayscale_image->height; ++row) {
            for (int col = 0; col < source_grayscale_image->width; ++col) {
                *bitmap_ptr++ = CV_IMAGE_ELEM(source_grayscale_image,
                        unsigned char, row, col);
            }
        }
        break;
    default:
        cvConvertScale(source_grayscale_image, m_Image_8U, 1);
        for (int row = 0; row < source_grayscale_image->height; ++row) {
            for (int col = 0; col < source_grayscale_image->width; ++col) {
                *bitmap_ptr++ = CV_IMAGE_ELEM(m_Image_8U,
                        unsigned char, row, col);
            }
        }
        break;
    }
////    float val;
//    for (int row = 0; row < source_grayscale_image->height; ++row) {
//        for (int col = 0; col < source_grayscale_image->width; ++col) {
////            val = CV_IMAGE_ELEM(source_grayscale_image, float, row, col);
////            if ((val < numeric_limits<short>::max()) || (val > numeric_limits<short>::min())) {
////                cout << val << endl;
////                assert(false);
////            }
//            *bitmap_ptr++ = static_cast<short>(
//                    CV_IMAGE_ELEM(source_grayscale_image, float, row, col));
//        }
//    }
    m_Pyramid1.build(&bitmap[0]);

//    static int counter = 1;
//    dump_pyramid(m_Pyramid1, &bitmap[0], "pyramid1/", counter);
//    dump_pyramid(m_Pyramid2, &bitmap[0], "pyramid2/", counter);
//    counter++;

    reestimate4roi(roi);

    m_PreviousPyramidBuilt = true;

} // recompute_image

/* static */ void
ip_Dense2DMotionProcessorInria::dump_pyramid(CMotion2DPyramid& pyramid,
        unsigned char * image, const std::string& prefix, int counter) {

    TPyramidError error;
    int nlevels = pyramid.getNumberOfLevels();
    int ncols;
    int nrows;

    ostringstream oss;

    IplImage * source_grayscale_image = getImageProvider()->image_buffer();
    unsigned char * source_image = image;
    ncols = source_grayscale_image->width;
    nrows = source_grayscale_image->height;
    IplImage * dump_source = cvCreateImage(cvSize(ncols, nrows),
            IPL_DEPTH_8U, 1);
    for (int row = 0; row < nrows; ++row) {
        for (int col = 0; col < ncols; ++col) {
            CV_IMAGE_ELEM(dump_source, unsigned char, row, col) =
                    *source_image++;
        }
    }
    oss << prefix << "source_" << setw(10) << setfill('0') << counter << ".png";
    cvSaveImage(oss.str().c_str(), dump_source);
    oss.str("");
    cvReleaseImage(&dump_source);


    for (int level = 0; level < nlevels; ++level) {
        pyramid.getNumberOfCols(ncols, level);
        pyramid.getNumberOfRows(nrows, level);
        short* data_gaussian = pyramid.getGaussianImageDataAddress(
                error, level);
        float* data_grad_col = pyramid.getColsSpatialGradientDataAddress(
                error, level);
        float* data_grad_row = pyramid.getRowsSpatialGradientDataAddress(
                error, level);

        IplImage * dump_gaussian = cvCreateImage(cvSize(ncols, nrows),
                IPL_DEPTH_16S, 1);
        IplImage * dump_gradient_col = cvCreateImage(cvSize(ncols, nrows),
                IPL_DEPTH_32F, 1);
        IplImage * dump_gradient_row = cvCreateImage(cvSize(ncols, nrows),
                IPL_DEPTH_32F, 1);
        for (int row = 0; row < nrows; ++row) {
            for (int col = 0; col < ncols; ++col) {
                CV_IMAGE_ELEM(dump_gaussian, short, row, col) =
                        *data_gaussian++;
                CV_IMAGE_ELEM(dump_gradient_col, float, row, col) =
                        *data_grad_col++;
                CV_IMAGE_ELEM(dump_gradient_row, float, row, col) =
                        *data_grad_row++;
            }
        }

        oss << prefix << "gaussian_l" << level << '_' << setw(10) << setfill('0')
            << counter << ".png";
        cvSaveImage(oss.str().c_str(), dump_gaussian);
        oss.str("");

        oss << prefix << "gradcol_l" << level << '_' << setw(10) << setfill('0')
            << counter << ".png";
        cvSaveImage(oss.str().c_str(), dump_gradient_col);
        oss.str("");

        oss << prefix << "gradrow_l" << level << '_' << setw(10) << setfill('0')
            << counter << ".png";
        cvSaveImage(oss.str().c_str(), dump_gradient_row);
        oss.str("");

        cvReleaseImage(&dump_gaussian);
        cvReleaseImage(&dump_gradient_col);
        cvReleaseImage(&dump_gradient_row);
    }

} // dump_pyramid

/* static */ void
ip_Dense2DMotionProcessorInria::dump_estimator(const ip_RoiWindow& roi,
        double origin_row, double origin_column,
        CMotion2DImage<unsigned char>& support,
        const std::string& prefix, int counter) {

    ostringstream oss;

    TPyramidError error;
    int ncols;
    int nrows;

    short* data_gaussian = m_Pyramid1.getGaussianImageDataAddress(error, 0);
    m_Pyramid1.getNumberOfCols(ncols, 0);
    m_Pyramid1.getNumberOfRows(nrows, 0);

    IplImage * dump_gaussian = cvCreateImage(cvSize(ncols, nrows),
            IPL_DEPTH_16S, 1);
    cvZero(dump_gaussian);

    for (int row = 0; row < nrows; ++row) {
        for (int col = 0; col < ncols; ++col) {
            if (support[row][col] > 0) {
                CV_IMAGE_ELEM(dump_gaussian, short, row, col) =
                        data_gaussian[row * ncols + col];
            }
        }
    }

    oss << prefix << "estimator_" << setw(10) << setfill('0') << counter << ".png";
    cvSaveImage(oss.str().c_str(), dump_gaussian);
    oss.str("");
    cvReleaseImage(&dump_gaussian);
} // dump_estimator

} // namespace ImageProcessing

#endif
