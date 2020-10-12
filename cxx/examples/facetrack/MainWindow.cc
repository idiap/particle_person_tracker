// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainWindow - class to visualise main model data
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                        // foreach loop
#include <boost/lambda/lambda.hpp>                  // lambda expressions
#include <numeric>                                  // for accumulate
#include <map>                                      // STL map
#include <iomanip>
#ifdef __STEREO_MATCHER_FOUND__
#include <colormap.hpp>
#include <image_processing/ip_DisparityImageProcessor.h>   // disparity/depth
#endif

// PROJECT INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // parameters2roi converter
#include <image_processing/ip_Dense2DMotionProcessorInria.h>

// LOCAL INCLUDES
#include "MainWindow.h"                                // declaration of this

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace BICV;
using namespace VFOA;

/////////////////////////// GLOBAL CONSTANTS /////////////////////////////////

static vector<CvScalar> initialize_vfoa_tracker_colours();
static vector<CvScalar> initialize_vfoa_object_colours();

static const int MAIN_WINDOW_WIDTH = 320;
static const int MAIN_WINDOW_HEIGHT = 240;
static const int VFOA_BAR_WIDTH = 5;
static const int VFOA_BAR_OFFSET = 3;
static const int DISTRIBUTION_BAR_OFFSET = 3;
static const int TICK_LENGTH = 5;
static const int DISTR_BAR_SPACING = 0;
static const CvScalar TICK_COLOUR = CV_RGB(60, 60, 0);
static const CvScalar DISTR_BAR_COLOUR = CV_RGB(255, 191, 0);
static const int TICK_WIDTH = 2;
static const int OBJECT_NAME_OFFSET = 13;
static const bool SHOW_OBJECT_NUMBER = true;

// Render disparity image, not input image
//#define SHOW_DISPARITY_IMAGE_NOT_SOURCE
// Render tracker on the resulting image
#define SHOULD_RENDER_TRACKER
// Render motion vectors on the resulting image
//#define SHOW_MOTION
// Render motion properties on the resulting image
//#define SHOW_MOTION_PROPERTIES
// Render nod detections
#define SHOW_NOD_DETECTOR
// Render likelihood value for the rendered state on the resulting image
//#define SHOW_LIKELIHOOD
// Render tracker scene coordinates on the resulting image
//#define SHOW_SCENE_COORDINATES
// Render tracker head pose on the resulting image
//#define SHOW_HEAD_POSE
// Render distributions over various parameters on the resulting image
//#define SHOW_DISTRIBUTIONS
// Render motion support using transparent colours
//#define SHOW_MOTION_SUPPORT
// Render global motion support using transparent colours
//#define SHOW_GLOBAL_MOTION_SUPPORT
// Render individual skin colour likelihoods
//#define SHOW_SKIN_COLOUR_LIKELIHOOD
// Render general skin colour likelihood
//#define SHOW_GENERAL_SKIN_COLOUR_CLASSIFICATION
// Render general skin colour likelihood
//#define SHOW_PERSONAL_SKIN_COLOUR_CLASSIFICATION
// Render all particles
//#define SHOW_ALL_PARTICLES
// Render face rectangles
#define SHOW_DETECTED_FACES
// Render tracker performance in FPS
#define SHOW_FPS

static const vector<CvScalar> VFOA_TARGET_COLOURS =
        initialize_vfoa_tracker_colours();
static const vector<CvScalar> VFOA_OBJECT_COLOURS =
        initialize_vfoa_object_colours();

/////////////////////////////// PUBLIC ///////////////////////////////////////

MainWindow::MainWindow(const MainModel& model):
  mIsInitialized(false),
  mModel(model),
  mImage8u3(0),
  mImageBuf8u3(0)
#ifdef __STEREO_MATCHER_FOUND__
    , m_DepthImage64f1(0)
    , m_DisparityRenderedImage32f3(0), m_DisparityRenderedImage8u3(0)
    , m_DisparityRenderedImage8u1(0)
#endif
{
}


MainWindow::
~MainWindow()
{
  if(mImage8u3)
    cvReleaseImage(&mImage8u3);

  if(mImageBuf8u3)
    cvReleaseImage(&mImageBuf8u3);

#ifdef __STEREO_MATCHER_FOUND__
    if (m_DisparityRenderedImage32f3) {
        cvReleaseImage(&m_DisparityRenderedImage32f3);
    }
    if (m_DisparityRenderedImage8u3) {
        cvReleaseImage(&m_DisparityRenderedImage8u3);
    }
    if (m_DisparityRenderedImage8u1) {
        cvReleaseImage(&m_DisparityRenderedImage8u1);
    }
    if (m_DepthImage64f1) {
        cvReleaseImage(&m_DepthImage64f1);
    }
#endif

}

void
MainWindow::
initialize()
{
  if(mIsInitialized) return;

  std::cout << "[MainWindow] Initializing with size "
            << mModel.data_provider()->image_buffer()->width << "x"
            << mModel.data_provider()->image_buffer()->height
            << std::endl;

  mImage8u3 =
    cvCreateImage(cvSize(mModel.data_provider()->image_buffer()->width,
                         mModel.data_provider()->image_buffer()->height),
                  IPL_DEPTH_8U, 3);

  mImageBuf8u3 =
    cvCreateImage(cvSize(mModel.data_provider()->image_buffer()->width,
                         mModel.data_provider()->image_buffer()->height),
                  IPL_DEPTH_8U, 3);

#ifdef __STEREO_MATCHER_FOUND__
  ip_DisparityImageProcessor * disparity_processor =
    dynamic_cast<ip_DisparityImageProcessor*>(mModel.disparity_processor());

  if(disparity_processor)
    {
      m_DepthImage64f1 = cvCreateImage(cvSize(disparity_processor->disparity_image()->width,
                                              disparity_processor->disparity_image()->height),
                                       IPL_DEPTH_64F, 1 );
      m_DisparityRenderedImage32f3 = cvCreateImage(cvSize(disparity_processor->disparity_image()->width,
                                                          disparity_processor->disparity_image()->height),
                                                   IPL_DEPTH_32F, 3 );
      m_DisparityRenderedImage8u3 = cvCreateImage(cvSize(disparity_processor->disparity_image()->width,
                                                         disparity_processor->disparity_image()->height),
                                                  IPL_DEPTH_8U, 3 );
      m_DisparityRenderedImage8u1 = cvCreateImage(cvSize(disparity_processor->disparity_image()->width,
                                                         disparity_processor->disparity_image()->height),
                                                  IPL_DEPTH_8U, 1 );
    }
#endif

  mIsInitialized = true;
}

const IplImage*
MainWindow::
update()
{
  CvFont font;
  cvInitFont(&font, CV_FONT_VECTOR0, 1.0, 1.0, 0.0, 1.0);

  // show original image
  IplImage *image = mModel.data_provider()->image();

  if(!image) return 0;

  if(!mIsInitialized) initialize();

#ifdef SHOW_DISPARITY_IMAGE_NOT_SOURCE
    #ifdef __STEREO_MATCHER_FOUND__
        ip_DisparityImageProcessor * disparity_processor =
                dynamic_cast<ip_DisparityImageProcessor*>(
                mModel.disparity_processor());
        if (disparity_processor) {
            disparity_processor->image();
            if (disparity_processor->has_valid_disparity()) {
                IplImage * disparity_image_32F1 =
                        disparity_processor->disparity_image();
                IplImage * xyz_image_64F3 =
                        disparity_processor->depth_image();

                {
                // show depth
                cvSetImageCOI(xyz_image_64F3, 3);
                cvCopy(xyz_image_64F3, m_DepthImage64f1);
                cvSetImageCOI(xyz_image_64F3, 0);
                cvConvertScale(m_DepthImage64f1, m_DisparityRenderedImage8u1, 0.05);
                cv::Mat disparity_mat_8u1(m_DisparityRenderedImage8u1);
                cv::Mat disparity_mat_32F3_temp;
                applyColorMap(disparity_mat_8u1, disparity_mat_32F3_temp, stereo::COLORMAP_JET);
                IplImage disparity_img_32F3_temp = disparity_mat_32F3_temp;
                cvConvertScale(&disparity_img_32F3_temp, m_DisparityRenderedImage8u3, 255);
//                cvShowImage("depth", m_DisparityRenderedImage8u3);
                }
//
//                {
//                // show disparity
//                cvConvertScale(disparity_image_32F1, m_DisparityRenderedImage8u1, 1);
//                cv::Mat disparity_mat_8u1(m_DisparityRenderedImage8u1);
//                cv::Mat disparity_mat_32F3_temp;
//                applyColorMap(disparity_mat_8u1, disparity_mat_32F3_temp, stereo::COLORMAP_JET);
//                IplImage disparity_img_32F3_temp = disparity_mat_32F3_temp;
//                cvConvertScale(&disparity_img_32F3_temp,
//                        m_DisparityRenderedImage8u3, 255);
//                cvShowImage("disparity", m_DisparityRenderedImage8u3);
//                }

                cvResize(m_DisparityRenderedImage8u3, mImageBuf8u3,
                        CV_INTER_LINEAR);
            } else {
//                cerr << "Disparity processor has invalid disparity!" << endl;
                cvConvertScale(image, mImageBuf8u3, 1.0);
            }
        } else {
            cerr << "Disparity processor is null!" << endl;
            cvConvertScale(image, mImageBuf8u3, 1.0);
        }
    #else  // __STEREO_MATCHER_FOUND__
        cvConvertScale(image, mImageBuf8u3, 1.0);
    #endif // __STEREO_MATCHER_FOUND__
#else  // SHOW_DISPARITY_IMAGE_NOT_SOURCE
    cvConvertScale(image, mImageBuf8u3, 1.0);
#endif // SHOW_DISPARITY_IMAGE_NOT_SOURCE

#ifdef SHOW_DETECTED_FACES
    show_detected_faces(mImageBuf8u3);
#endif

    // fill colour maps for VFOA targets

    map<unsigned, CvScalar> objId2Colour;
#ifdef __VFOA_MODULE_FOUND__
//    cout << "Vfoa module person list: (";
    VfoaManager * vfoa_model = mModel.vfoa_model();
    identified_distribution_list_t id_distr_list = vfoa_model->GetVfoaList();
    BOOST_FOREACH(const identified_distribution_t& id_distr, id_distr_list) {
        objId2Colour[id_distr.id] = VFOA_TARGET_COLOURS[
            id_distr.id % VFOA_TARGET_COLOURS.size()];
//        cout << id_distr.id << ", ";
    }
//    cout << ")" << endl;
#else
    const vfoa_CognitiveVfoaModel * vfoa_model = mModel.vfoa_model();
    list<vfoa_CognitiveVfoaModelObjectInfo> targets = vfoa_model->objects();
    unsigned idx = 0;
    BOOST_FOREACH(const vfoa_CognitiveVfoaModelObjectInfo& obj_info, targets) {
        objId2Colour[obj_info.mId] = VFOA_TARGET_COLOURS[
                                           idx % VFOA_TARGET_COLOURS.size()];
        ++idx;
    }
#endif

//    FaceColorModel::FaceColorModel * face_color_model =
//            mModel.general_face_color_model();
//    vector<double> buffer(mImageBuf8u3->width * mImageBuf8u3->height *
//            FaceColorModel::FCM_NUM_CHANNELS, 0);
//    face_color_model->channel_log_likelihood(mImageBuf8u3, &buffer[0]);
//
//    cvZero(mImageBuf8u3);
//
//    double log_lhood_skin, proba_skin;
//    double log_lhood_hair, proba_hair;
//    double log_lhood_clothes, proba_clothes;
//    double log_lhood_bg, proba_bg;
//    double max_log_lhood, sum_proba;
//
//    for(int row = 0; row < mImageBuf8u3->height; row++) {
//        for(int col = 0; col < mImageBuf8u3->width; col++) {
//            log_lhood_skin = buffer[row * mImageBuf8u3->width + col];
//            log_lhood_hair = buffer[(mImageBuf8u3->height + row) *
//                                    mImageBuf8u3->width + col];
//            log_lhood_clothes = buffer[(2 * mImageBuf8u3->height + row) *
//                                    mImageBuf8u3->width + col];
//            log_lhood_bg =   buffer[(3 * mImageBuf8u3->height + row) *
//                                    mImageBuf8u3->width + col];
//            max_log_lhood = max(max(log_lhood_skin, log_lhood_hair),
//                    max(log_lhood_clothes, log_lhood_bg));
//            proba_skin = exp(log_lhood_skin - max_log_lhood);
//            proba_hair = exp(log_lhood_hair - max_log_lhood);
//            proba_clothes = exp(log_lhood_clothes - max_log_lhood);
//            proba_bg = exp(log_lhood_bg - max_log_lhood);
//            sum_proba = proba_skin + proba_hair + proba_clothes + proba_bg;
//            proba_skin /= sum_proba;
//            proba_hair /= sum_proba;
//            proba_clothes /= sum_proba;
//            proba_bg /= sum_proba;
//
//            CV_IMAGE_ELEM(mImageBuf8u3, uchar, row, 3 * col) = proba_clothes * 255;
//            CV_IMAGE_ELEM(mImageBuf8u3, uchar, row, 3 * col + 1) = proba_hair * 255;
//            CV_IMAGE_ELEM(mImageBuf8u3, uchar, row, 3 * col + 2) = proba_skin * 255;
//        }
//    }

    const list<bicv_HeadPoseTracker*>& trackers = mModel.head_pose_trackers();

#if (defined SHOW_MOTION_SUPPORT || defined SHOW_SKIN_COLOUR_LIKELIHOOD)
    const float tracker_coeff = 1.0f / (trackers.size() + 1);
#endif

#ifdef SHOW_GENERAL_SKIN_COLOUR_CLASSIFICATION
    show_general_skin_colour_classification(image, mImageBuf8u3);
#else
#ifdef SHOW_PERSONAL_SKIN_COLOUR_CLASSIFICATION
    BOOST_FOREACH (bicv_HeadPoseTracker * tracker, trackers) {
        show_personal_skin_colour_classification(image, mImageBuf8u3,
                tracker);
	break;
    }
#endif
#endif

#ifdef SHOW_GLOBAL_MOTION_SUPPORT
        show_global_motion_support(mImageBuf8u3);
#endif

    BOOST_FOREACH(bicv_HeadPoseTracker * tracker, trackers) {
//        cout << "Start render tracker " << tracker << ", time " <<
//                boost::posix_time::microsec_clock::local_time() << endl;
#ifdef SHOULD_RENDER_TRACKER
        render_tracker(tracker, mImageBuf8u3, objId2Colour);
#endif
//        cout << "End render tracker " << tracker << ", time " <<
//                boost::posix_time::microsec_clock::local_time() << endl;

#ifdef SHOW_MOTION_SUPPORT
//        show_motion_support(tracker, mImageBuf8u3, objId2Colour, tracker_coeff);
        show_motion_support(tracker, mImageBuf8u3, objId2Colour, 1.0);
#endif
#ifdef SHOW_SKIN_COLOUR_LIKELIHOOD
        show_skin_colour_likelihood(tracker, mImageBuf8u3, objId2Colour,
            tracker_coeff);
#endif
    }

#ifdef SHOW_FPS
    show_fps(mImageBuf8u3);
#endif

    cvResize( mImageBuf8u3, mImage8u3, CV_INTER_LINEAR );

    // std::cout << "[MainWindow] update() finished "
    //           << to_simple_string(mModel.data_provider()->time()) << std::endl;

    return mImage8u3;
}

void MainWindow::show_skin_colour_likelihood(bicv_HeadPoseTracker * tracker,
        IplImage * image8u3,
        std::map<unsigned, CvScalar>& objId2ColourMap,
        float coeff) {

#ifdef __VFOA_MODULE_FOUND__
    CvScalar obj_colour = objId2ColourMap[tracker->id()];
#else
    CvScalar obj_colour = objId2ColourMap[
           mModel.vfoa_model()->object_info(tracker).mId];
#endif

    ip_SkinColourProcessor * scp = tracker->skin_colour_processor();
    IplImage * scp_image = scp->image();
//    cout << "coeff=" << coeff << ", colour (" <<
//         obj_colour.val[0] << ", " << obj_colour.val[1] << ", "<<
//         obj_colour.val[2] << ")" << endl;

    float lhood;
    float maxlhood = 0;
    int maxlhood_col = 0;
    int maxlhood_row = 0;
    for (int row = 0; row < image8u3->height; ++row) {
        for (int col = 0; col < image8u3->width; ++col) {
            lhood = CV_IMAGE_ELEM(scp_image, uchar, row, col);
            if (lhood > maxlhood) {
                maxlhood = lhood;
                maxlhood_col = col;
                maxlhood_row = row;
            }
            CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col) =
                    min(CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col) +
                    coeff * obj_colour.val[0] * lhood, 255.0);
            CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col + 1) =
                    min(CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col + 1) +
                    coeff * obj_colour.val[1] * lhood, 255.0);
            CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col + 2) =
                    min(CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col + 2) +
                    coeff * obj_colour.val[2] * lhood, 255.0);
        }
    }
//    cout << "Max lhood: " << maxlhood << ", col=" << maxlhood_col <<
//            ", row=" << maxlhood_row << endl;
} // show_colour_model_likelihoods

void MainWindow::show_general_skin_colour_classification(IplImage * src_image8u3,
        IplImage * dst_image8u3) {

    ip_RoiWindow global_roi(0, 0, src_image8u3->width, src_image8u3->height);

    FaceColorModel::FaceColorModel * colour_model = mModel.general_face_color_model();
    show_skin_colour_classification(src_image8u3, dst_image8u3, colour_model,
        global_roi);

} // show_general_skin_colour_classification

void MainWindow::show_personal_skin_colour_classification(
        IplImage * src_image8u3, IplImage * dst_image8u3,
        BICV::bicv_HeadPoseTracker * tracker) {

    FaceColorModel::FaceColorModel * colour_model = tracker->face_color_model();
    bicv_HeadPoseARTrackerState shown_state = mean(
            tracker->particle_distribution());
    ip_RoiWindow main_roi = HpParams2RoiConverter::hpparams2roi(
            shown_state.m_HeadPoseParamsCur);
    ip_RoiWindow fcm_roi = ip_RoiWindow::from_CvRect(
            colour_model->face_roi2fcm_roi(ip_RoiWindow::to_CvRect(main_roi)));
    show_skin_colour_classification(src_image8u3, dst_image8u3, colour_model,
            ip_RoiWindow(0, 0, src_image8u3->width, src_image8u3->height));
} // show_personal_skin_colour_classification

void MainWindow::show_skin_colour_classification(
        IplImage * src_image8u3, IplImage * dst_image8u3,
        FaceColorModel::FaceColorModel * colour_model,
        const ip_RoiWindow& roi) {

    ip_RoiWindow global_roi(0, 0, src_image8u3->width, src_image8u3->height);
    ip_RoiWindow v_roi;
    if (!intersect(global_roi, roi, v_roi)) {
        return;
    }

    colour_model->cache_probability_maps(src_image8u3, ip_RoiWindow::to_CvRect(v_roi));
    const FaceColorModel::FaceColorModel::PimFeatureType * proba_maps =
            colour_model->cached_probability_maps();

    typedef FaceColorModel::FaceColorModel::PimFeatureType::value_type PType;

    using namespace FaceColorModel;

    vector<CvScalar> class_colours(FCM_NUM_CHANNELS);
    class_colours[FCM_CHANNEL_SKIN] = CV_RGB(255, 255,   0);
    class_colours[FCM_CHANNEL_HAIR] = CV_RGB(255,   0,   0);
    class_colours[FCM_CHANNEL_CLOTHES] = CV_RGB(  0, 255,   0);
    class_colours[FCM_CHANNEL_BACKGROUND] = CV_RGB(  0,   0,  0);

    CvScalar colour_default = CV_RGB( 64,  64,  64);
    CvScalar colour;

    vector<fcm_real> class_log_lhoods(FCM_NUM_CHANNELS);

    for (int row = v_roi.m_iFirstRow; row < v_roi.m_iFirstRow + v_roi.m_iHeight; ++row) {
        for (int col = v_roi.m_iFirstColumn; col < v_roi.m_iFirstColumn + v_roi.m_iWidth; ++col) {
            class_log_lhoods[FCM_CHANNEL_SKIN] =
                CV_MAT_ELEM(*(proba_maps->map(FCM_CHANNEL_SKIN)), PType, row, col);
            class_log_lhoods[FCM_CHANNEL_HAIR] =
                CV_MAT_ELEM(*(proba_maps->map(FCM_CHANNEL_HAIR)), PType, row, col);
            class_log_lhoods[FCM_CHANNEL_CLOTHES] =
                CV_MAT_ELEM(*(proba_maps->map(FCM_CHANNEL_CLOTHES)), PType, row, col);
            class_log_lhoods[FCM_CHANNEL_BACKGROUND] =
                CV_MAT_ELEM(*(proba_maps->map(FCM_CHANNEL_BACKGROUND)), PType, row, col);
//            cout << class_log_lhoods[FCM_CHANNEL_SKIN] << " "
//                 << class_log_lhoods[FCM_CHANNEL_HAIR] << " "
//                 << class_log_lhoods[FCM_CHANNEL_CLOTHES] << " "
//                 << class_log_lhoods[FCM_CHANNEL_BACKGROUND] << endl;

            vector<fcm_real>::iterator ii = max_element(class_log_lhoods.begin(),
                    class_log_lhoods.end());

            switch(ii - class_log_lhoods.begin()) {
            case FCM_CHANNEL_SKIN:
            case FCM_CHANNEL_HAIR:
            case FCM_CHANNEL_CLOTHES:
            case FCM_CHANNEL_BACKGROUND:
                colour = class_colours[ii - class_log_lhoods.begin()];
                break;
            default:
                colour = colour_default;
            }

            CV_IMAGE_ELEM(dst_image8u3, uchar, row, 3 * col) = colour.val[0];
            CV_IMAGE_ELEM(dst_image8u3, uchar, row, 3 * col + 1) = colour.val[1];
            CV_IMAGE_ELEM(dst_image8u3, uchar, row, 3 * col + 2) = colour.val[2];

        }
    }
} // show_skin_colour_classification

void MainWindow::show_global_motion_support(IplImage * image8u3) {
    ip_Dense2DMotionProcessorInria * motion_proc =
            dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                    mModel.dense_motion_processor());
    ip_RoiWindow roi;
    roi.m_iFirstColumn = 0;
    roi.m_iFirstRow = 0;
    roi.m_iHeight = mModel.data_provider()->image_buffer()->height;
    roi.m_iWidth = mModel.data_provider()->image_buffer()->width;
    motion_proc->reestimate4roi(roi);

    //CMotion2DEstimator * motion_estimator = motion_proc->getEstimator();

} // show_global_motion_support

void MainWindow::show_motion_support(BICV::bicv_HeadPoseTracker * tracker,
        IplImage * image8u3,
        std::map<unsigned, CvScalar>& objId2ColourMap, float coeff) {

#ifdef __VFOA_MODULE_FOUND__
    CvScalar obj_colour = objId2ColourMap[tracker->id()];
#else
    CvScalar obj_colour = objId2ColourMap[
           mModel.vfoa_model()->object_info(tracker).mId];
#endif

    ip_Dense2DMotionProcessorInria * motion_proc =
            dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                    mModel.dense_motion_processor());
    CMotion2DEstimator * motion_estimator = motion_proc->getEstimator();

    int nrows;
    int ncols;
    float * weights = motion_estimator->getWeights(nrows, ncols);

    if ((nrows > 0) && (ncols > 0)) {
        IplImage * temp_image = cvCreateImage( cvSize(ncols, nrows),
                IPL_DEPTH_8U, 3 );
        cvZero(temp_image);

        float weight;
        for (int row = 0; row < nrows; ++row) {
            for (int col = 0; col < ncols; ++col) {
                weight = weights[row * ncols + col];
                CV_IMAGE_ELEM(temp_image, uchar, row, 3 * col) =
                        min(CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col) +
                        coeff * obj_colour.val[0] * weight, 255.0);
                CV_IMAGE_ELEM(temp_image, uchar, row, 3 * col + 1) =
                        min(CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col + 1) +
                        coeff * obj_colour.val[1] * weight, 255.0);
                CV_IMAGE_ELEM(temp_image, uchar, row, 3 * col + 2) =
                        min(CV_IMAGE_ELEM(image8u3, uchar, row, 3 * col + 2) +
                        coeff * obj_colour.val[2] * weight, 255.0);
            }
        }

        cvResize(temp_image, image8u3);

        cvReleaseImage(&temp_image);
    }

} // show_motion_support


void MainWindow::show_detected_faces(IplImage * image8u3) {
    CvFont screenOutputFont;
    cvInitFont(&screenOutputFont, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0, 2.);
    CvSize textSize;
    int baseline = 0;
    cvGetTextSize("Xyd", &screenOutputFont,
            &textSize, &baseline);
    CvScalar face_detection_colour = CV_RGB(255, 0, 255);

    const list<cvp_FaceDescriptor>& faces = mModel.faces_cache();
    BOOST_FOREACH(cvp_FaceDescriptor fd, faces) {
        cvRectangle(image8u3, cvPoint(fd.m_FaceRegion.x,
                fd.m_FaceRegion.y),
            cvPoint(fd.m_FaceRegion.x + fd.m_FaceRegion.width,
                    fd.m_FaceRegion.y    + fd.m_FaceRegion.height),
            face_detection_colour,
            2);
        switch(fd.m_Pose) {
        case CVP_FACEDETECTOR_FACE:
            cvPutText(image8u3, "F",
                    cvPoint(fd.m_FaceRegion.x + fd.m_FaceRegion.width / 2,
                            fd.m_FaceRegion.y - baseline),
                    &screenOutputFont,
                    face_detection_colour);
            break;
        case CVP_FACEDETECTOR_PROFILE_LEFT:
            cvPutText(image8u3, "L",
                    cvPoint(fd.m_FaceRegion.x,
                            fd.m_FaceRegion.y - baseline),
                    &screenOutputFont,
                    face_detection_colour);
            break;
        case CVP_FACEDETECTOR_PROFILE_RIGHT:
            cvPutText(image8u3, "R",
                    cvPoint(fd.m_FaceRegion.x + fd.m_FaceRegion.width,
                            fd.m_FaceRegion.y - baseline),
                    &screenOutputFont,
                    face_detection_colour);
            break;
        case CVP_FACEDETECTOR_POSE_UNKNOWN:
            cvPutText(image8u3, "U",
                    cvPoint(fd.m_FaceRegion.x + fd.m_FaceRegion.width / 2,
                            fd.m_FaceRegion.y - baseline),
                    &screenOutputFont,
                    face_detection_colour);
            break;
        }
    }
}

void MainWindow::show_fps(IplImage * image8u3) {
    CvFont screenOutputFont;
    cvInitFont(&screenOutputFont, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0, 2.);

    float fps = mModel.performance_fps();
    ostringstream oss;
    oss << std::setprecision(1);
    oss << std::fixed;
    oss << fps << " fps";
    string fps_str = oss.str();

    CvSize textSize;
    int baseline = 0;
    cvGetTextSize(fps_str.c_str(), &screenOutputFont,
            &textSize, &baseline);

    cvPutText(image8u3, oss.str().c_str(),
            cvPoint(image8u3->width - textSize.width, textSize.height),
            &screenOutputFont,
            CV_RGB(255, 0, 0));
}

void MainWindow::render_tracker(bicv_HeadPoseTracker * tracker,
        IplImage * image8u3,
        std::map<unsigned, CvScalar>& objId2ColourMap) {

    CvFont screenOutputFont;
    cvInitFont(&screenOutputFont, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0, 2.);
    CvFont black_font;
    cvInitFont(&black_font, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0.0, 3.0);
    CvFont system_font;
    cvInitFont(&system_font, CV_FONT_HERSHEY_PLAIN, .8, .8, 0.0, 1.0);

    // render mode
    bicv_HeadPoseARTrackerState shown_state = mean(
            tracker->particle_distribution());
    ip_RoiWindow main_roi = HpParams2RoiConverter::hpparams2roi(
            shown_state.m_HeadPoseParamsCur);

    CvScalar obj_colour;

#ifdef __VFOA_MODULE_FOUND__
    if (objId2ColourMap.find(tracker->id()) == objId2ColourMap.end()) {
        cout << "Could not find colour for object " << tracker->id() << endl;
    } else {
        obj_colour = objId2ColourMap[tracker->id()];
    }
#else
    obj_colour = objId2ColourMap[
           mModel.vfoa_model()->object_info(tracker).mId];
#endif
    cvRectangle(image8u3, cvPoint(main_roi.m_iFirstColumn, main_roi.m_iFirstRow),
        cvPoint(main_roi.m_iFirstColumn + main_roi.m_iWidth,
                main_roi.m_iFirstRow + main_roi.m_iHeight), obj_colour, 2);

    // render motion
#ifdef SHOW_MOTION
    ImageProcessing::ip_Dense2DMotionProcessor * motion_proc =
            mModel.dense_motion_processor();
    ip_RoiWindow roi = main_roi;
//    cout << "Mean state ROI: " << main_roi << endl;
    double support_scale = 1.2;
    roi.m_iFirstColumn = max(static_cast<int>(roi.m_iFirstColumn +
            roi.m_iWidth / 2 - support_scale * roi.m_iWidth / 2), 0);
    roi.m_iFirstRow = max(static_cast<int>(roi.m_iFirstRow +
            roi.m_iHeight / 2 - support_scale * roi.m_iHeight / 2), 0);
    roi.m_iHeight = roi.m_iHeight * support_scale;
    if (roi.m_iFirstColumn + roi.m_iWidth > image8u3->width) {
        roi.m_iWidth = image8u3->width - roi.m_iFirstColumn;
    }
    roi.m_iWidth = roi.m_iWidth * support_scale;
    if (roi.m_iFirstRow + roi.m_iHeight > image8u3->height) {
        roi.m_iHeight = image8u3->height - roi.m_iFirstRow;
    }

    motion_proc->image(roi); // recomputes pyramides if necessary
    motion_proc->reestimate4roi(roi); // recomputes results for ROI
    IplImage * imgX = motion_proc->getMotionXProvider()->image(roi);
    IplImage * imgY = motion_proc->getMotionYProvider()->image(roi);

    for (int ri = roi.m_iFirstRow; ri <= roi.m_iFirstRow + roi.m_iHeight; ri += 10) {
        for (int rj = roi.m_iFirstColumn; rj <= roi.m_iFirstColumn + roi.m_iWidth; rj += 10) {
            float dx = CV_IMAGE_ELEM(imgX, float, ri, rj);
            float dy = CV_IMAGE_ELEM(imgY, float, ri, rj);
            cvLine(image8u3, cvPoint(rj, ri), cvPoint(rj + dx, ri + dy),
                    cvScalar(0, 255, 0), 0.1f);
            cvCircle(image8u3, cvPoint(rj, ri), 1, cvScalar(0, 255, 0), 1);
        }
    }
#endif

    {

        ostringstream oss;
        oss << "dxy";
        string motion_prop_str = oss.str();

        CvSize textSize;
        int baseline = 0;
        cvGetTextSize(motion_prop_str.c_str(), &screenOutputFont,
                &textSize, &baseline);

        ip_RoiWindow roi_text;
        roi_text.m_iFirstColumn = min(max(static_cast<int>(
                round(main_roi.m_iFirstColumn)), 0),
                image8u3->width - 1);
        roi_text.m_iFirstRow = min(max(main_roi.m_iFirstRow +
                main_roi.m_iHeight + 10, 0), image8u3->height - 1);
        roi_text.m_iWidth = min(textSize.width + OBJECT_NAME_OFFSET, image8u3->width - roi_text.m_iFirstColumn);
        roi_text.m_iHeight = min( textSize.height + OBJECT_NAME_OFFSET,
                image8u3->height - roi_text.m_iFirstRow);
        int sys_offset = roi_text.m_iFirstRow;
        int num_fld_width = 8;

#ifdef SHOW_LIKELIHOOD
        const bicv_HeadPoseTracker::Likelihood * lhood_model =
                tracker->likelihood();
        ip_HogFeatureProducerRounded * hfp = tracker->hog_feature_producer();
        const ip_HistogramTemplate& hog_feature = hfp->compute_feature(main_roi);

        ip_SkinFeatureProducer * sfp = tracker->skin_feature_producer();
        const ip_SkinTemplate& skin_feature = sfp->compute_feature(main_roi);

        bicv_SkinHogObservation skinhog_obs(hog_feature, skin_feature);
        float lhood = lhood_model->evaluate(shown_state, skinhog_obs);
        oss.str("");
        oss << "likelihood " << setw(num_fld_width) << lhood;
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height * 4)),
                &system_font,
                CV_RGB(255, 0, 0));
        sys_offset += textSize.height;
#endif

#ifdef SHOW_SCENE_COORDINATES
        CvScalar tracker_coords = tracker->scene_coordinates_3D();
        oss.str("");
        oss << "Location (";
        oss.precision(2);
        oss << fixed;
        ostream_iterator<double> oss_iter(oss, ", ");
        copy(tracker_coords.val, tracker_coords.val + 3, oss_iter);
        oss << ")";
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height * 4)),
                &system_font,
                CV_RGB(255, 0, 0));
        sys_offset += textSize.height;
#endif

#ifdef SHOW_HEAD_POSE
        {
        bicv_HeadPoseARTrackerState mean_tracker_state =
                mean(tracker->particle_distribution());
        cvp_HeadPoseDiscreteDomain::HeadPose mean_tracker_head_pose =
                mean_tracker_state.m_HeadPoseParamsCur.m_HeadPose;
        oss.str("");
        oss << "Mean HP (";
        oss.precision(2);
        oss << fixed;
        oss << mean_tracker_head_pose.pan() << ", ";
        oss << mean_tracker_head_pose.tilt() << ", ";
        oss << mean_tracker_head_pose.roll() << ")";
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height * 4)),
                &system_font,
                CV_RGB(255, 0, 0));
        sys_offset += textSize.height;
        }
#endif

#ifdef SHOW_DISTRIBUTIONS
        {
            const boost::math::normal_distribution<float>& scale_prior =
                    tracker->scale_prior();
        oss.str("");
        oss << "Scale prior (";
        oss.precision(2);
        oss << fixed;
        oss << scale_prior.mean() << ", ";
        oss << scale_prior.standard_deviation() << ")";
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height * 4)),
                &system_font,
                CV_RGB(255, 0, 0));
        sys_offset += textSize.height;
        }
#endif

#ifdef SHOW_MOTION_PROPERTIES
        ip_Dense2DMotionProcessorInria * motion_proc_inria =
                dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                        mModel.dense_motion_processor());
        CMotion2DModel * motion_model = motion_proc_inria->getModel();
        double parameters[12];
        motion_model->getParameters(parameters);

        oss.str("");
        oss << "dx  " << showpos << setw(num_fld_width) << parameters[0];
        motion_prop_str = oss.str();

        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height)),
                &system_font,
                CV_RGB(255, 0, 0));

        sys_offset += textSize.height;
        oss.str("");
        oss << "dy  " << showpos << setw(num_fld_width) << parameters[1];
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height)),
                &system_font,
                CV_RGB(255, 0, 0));

        sys_offset += textSize.height;
        oss.str(""); oss << "div " << showpos << setw(num_fld_width) << parameters[2];
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height)),
                &system_font,
                CV_RGB(255, 0, 0));

        sys_offset += textSize.height;
        oss.str(""); oss << "rot " << showpos << setw(num_fld_width) << parameters[3];
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height)),
                &system_font,
                CV_RGB(255, 0, 0));

        double row, col;
        motion_model->getOrigin(row, col);

        sys_offset += textSize.height;
        oss.str(""); oss << "row " << showpos << setw(num_fld_width) << row;
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height)),
                &system_font,
                CV_RGB(255, 0, 0));

        sys_offset += textSize.height;
        oss.str(""); oss << "col " << showpos << setw(num_fld_width) << col;
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height)),
                &system_font,
                CV_RGB(255, 0, 0));
#endif
    }

//    for (int ri = 0; ri < image8u3->height; ri += 10) {
//        for (int rj = 0; rj < image8u3->width; rj += 10) {
//            float dx = CV_IMAGE_ELEM(imgX, float, ri, rj);
//            float dy = CV_IMAGE_ELEM(imgY, float, ri, rj);
//            cvLine(image8u3, cvPoint(rj, ri), cvPoint(rj + dx, ri + dy),
//                    cvScalar(0, 255, 0), 0.1f);
//        }
//    }

    // render mean
//    bicv_HeadPoseARTrackerState mean_state = mean(
//            tracker->particle_distribution());
//    ip_RoiWindow main_roi = HpParams2RoiConverter::hpparams2roi(
//            mean_state.m_HeadPoseParamsCur);
//    cvRectangle(image8u3, cvPoint(main_roi.m_iFirstColumn, main_roi.m_iFirstRow),
//        cvPoint(main_roi.m_iFirstColumn + main_roi.m_iWidth,
//                main_roi.m_iFirstRow + main_roi.m_iHeight), cvScalar(0, 255, 0),
//        2);

    if (SHOW_OBJECT_NUMBER) {
        CvSize textSize;
        int baseline = 0;

    #ifdef __VFOA_MODULE_FOUND__
//        CvScalar obj_colour = objId2ColourMap[tracker->id()];
        unsigned obj_number = tracker->id();
    #else
        vfoa_CognitiveVfoaModelObjectInfo obj_info =
                mModel.vfoa_model()->object_info(tracker);
        unsigned obj_number = obj_info.mNumber;
//        CvScalar obj_colour = objId2ColourMap[obj_info.mId];
    #endif

        ostringstream oss;
        oss << obj_number;
        string obj_number_str = oss.str();
        cvGetTextSize(obj_number_str.c_str(), &screenOutputFont,
                &textSize, &baseline);
        ip_RoiWindow roi_text;
        roi_text.m_iFirstColumn = min(max(static_cast<int>(
                round(main_roi.m_iFirstColumn - textSize.width -
                        OBJECT_NAME_OFFSET)), 0),
                image8u3->width - 1);
        roi_text.m_iFirstRow = min(max(main_roi.m_iFirstRow, 0),
                image8u3->height - 1);
        roi_text.m_iWidth = min(textSize.width + OBJECT_NAME_OFFSET,
                image8u3->width - roi_text.m_iFirstColumn);
        roi_text.m_iHeight = min(
                static_cast<int>(round(textSize.height + OBJECT_NAME_OFFSET)),
                image8u3->height - roi_text.m_iFirstRow);
        blend_rect(image8u3, roi_text);
        cvPutText(image8u3, obj_number_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn + 0.5 * OBJECT_NAME_OFFSET),
                        round(roi_text.m_iFirstRow + 0.5 * OBJECT_NAME_OFFSET +
                        textSize.height)),
                &black_font,
                CV_RGB(0, 0, 0));
        cvPutText(image8u3, obj_number_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn + 0.5 * OBJECT_NAME_OFFSET),
                        round(roi_text.m_iFirstRow + 0.5 * OBJECT_NAME_OFFSET +
                        textSize.height)),
                &screenOutputFont,
                obj_colour);
    }

#ifdef SHOW_ALL_PARTICLES
    typedef BayesFilter::bf_DiscreteDistribution<
        bicv_HeadPoseARTrackerState> ParticleDistribution;
    const ParticleDistribution& pdist = tracker->particle_distribution();
    ip_RoiWindow particle_roi;

    BOOST_FOREACH(const ParticleDistribution::element_type& elt,
            pdist.elements()) {
        particle_roi = HpParams2RoiConverter::hpparams2roi(
                elt.first.m_HeadPoseParamsCur);
        CvScalar colour = VFOA_TARGET_COLOURS[elt.first.m_Label %
                                              VFOA_TARGET_COLOURS.size()];
        cvRectangle(image8u3, cvPoint(particle_roi.m_iFirstColumn,
                particle_roi.m_iFirstRow),
            cvPoint(particle_roi.m_iFirstColumn + main_roi.m_iWidth - 1,
                    particle_roi.m_iFirstRow + main_roi.m_iHeight - 1),
                    colour, 1);
    }
#endif

    // render distributions
    typedef BayesFilter::bf_DiscreteDistribution<bicv_HeadPoseARTrackerState>::
        element_type ElementType;
    const cvp_HeadPoseDiscreteDomain& head_pose_domain =
            mModel.head_pose_domain();
    const std::vector<ElementType>& elements = tracker->
            particle_distribution().elements();
    vector<float> pan_weights(head_pose_domain.pan_values().size(), 0);
    vector<float> tilt_weights(head_pose_domain.tilt_values().size(), 0);
    BOOST_FOREACH(const ElementType& element, elements) {
        cvp_HeadPoseDiscreteDomain::HeadPose head_pose =
            element.first.m_HeadPoseParamsCur.m_HeadPose;
        head_pose = head_pose_domain.discretize(head_pose);
        pan_weights[head_pose_domain.pan_index(head_pose)] += element.second;
        tilt_weights[head_pose_domain.tilt_index(head_pose)] += element.second;
    }

    using namespace boost::lambda;
    float sum_pans = accumulate(pan_weights.begin(), pan_weights.end(), 0.0f);
    float sum_tilts = accumulate(tilt_weights.begin(), tilt_weights.end(), 0.0f);
    transform(pan_weights.begin(), pan_weights.end(), pan_weights.begin(),
            boost::lambda::_1 / sum_pans);
    transform(tilt_weights.begin(), tilt_weights.end(),
            tilt_weights.begin(), boost::lambda::_1 / sum_tilts);

//    cout << endl;
//    ostream_iterator<float> out_it(cout, ", ");
//    copy(pan_weights.begin(), pan_weights.end(), out_it);
//    cout << endl;

    // choose bar width for pan and tilt distributions
    static const float bar_spacing = DISTR_BAR_SPACING;
    const float pan_bar_width = static_cast<float>(main_roi.m_iWidth
            - bar_spacing * (pan_weights.size() - 1)) / pan_weights.size();
    const float tilt_bar_width = static_cast<float>(main_roi.m_iHeight
            - bar_spacing * (tilt_weights.size() - 1)) / tilt_weights.size();
    const float bar_width = min(pan_bar_width, tilt_bar_width);

    // render pan distribution
    const float pan_median = main_roi.m_iFirstColumn + 0.5 * main_roi.m_iWidth;
    const float pan_distr_half_len = 0.5 * (bar_width * pan_weights.size() +
            bar_spacing * (pan_weights.size() - 1));
    float offset = pan_median + pan_distr_half_len;
    const float pan_bar_row_start = main_roi.m_iFirstRow -
            DISTRIBUTION_BAR_OFFSET;
    for (unsigned i = 0; i < pan_weights.size(); ++i) {
        cvRectangle(image8u3,
                cvPoint(round(offset), round(pan_bar_row_start)),
                cvPoint(round(offset - pan_bar_width), round(pan_bar_row_start
                        - main_roi.m_iHeight * pan_weights[i])),
                DISTR_BAR_COLOUR, CV_FILLED);
        offset -= bar_width + bar_spacing;
    }

    // render pan ticks
    const float tick_row_end = pan_bar_row_start - TICK_LENGTH;
    // middle
    cvRectangle(image8u3,
            cvPoint(round(pan_median - 0.5 * TICK_WIDTH), round(pan_bar_row_start)),
            cvPoint(round(pan_median + 0.5 * TICK_WIDTH), round(tick_row_end)),
            obj_colour, CV_FILLED);
    // left
    cvRectangle(image8u3,
            cvPoint(round(pan_median - pan_distr_half_len - TICK_WIDTH),
                    round(pan_bar_row_start)),
            cvPoint(round(pan_median - pan_distr_half_len),
                    round(tick_row_end)),
            obj_colour, CV_FILLED);
    // right
    cvRectangle(image8u3,
            cvPoint(round(pan_median + pan_distr_half_len + TICK_WIDTH),
                    round(pan_bar_row_start)),
            cvPoint(round(pan_median + pan_distr_half_len),
                    round(tick_row_end)),
            obj_colour, CV_FILLED);


    // render tilt distribution
    const float tilt_median = main_roi.m_iFirstRow + 0.5 * main_roi.m_iHeight;
    const float tilt_distr_half_len = 0.5 * (bar_width * tilt_weights.size() +
            bar_spacing * (tilt_weights.size() - 1));
    offset = tilt_median + tilt_distr_half_len;
    const float tilt_bar_col_start = main_roi.m_iFirstColumn +
            main_roi.m_iWidth + DISTRIBUTION_BAR_OFFSET;
    for (unsigned i = 0; i < tilt_weights.size(); ++i) {
        cvRectangle(image8u3,
                cvPoint(round(tilt_bar_col_start), round(offset)),
                cvPoint(round(tilt_bar_col_start + main_roi.m_iWidth *
                        tilt_weights[i]), round(offset - bar_width)),
                DISTR_BAR_COLOUR, CV_FILLED);
        offset -= bar_width + bar_spacing;
    }

    // render tilt ticks
    const float tick_col_end = tilt_bar_col_start + TICK_LENGTH;
    // middle
    cvRectangle(image8u3,
            cvPoint(round(tilt_bar_col_start),
                    round(tilt_median - 0.5 * TICK_WIDTH)),
            cvPoint(round(tick_col_end),
                    round(tilt_median + 0.5 * TICK_WIDTH)),
            obj_colour, CV_FILLED);
    // top
    cvRectangle(image8u3,
            cvPoint(round(tilt_bar_col_start),
                    round(tilt_median - tilt_distr_half_len - TICK_WIDTH)),
            cvPoint(round(tick_col_end),
                    round(tilt_median - tilt_distr_half_len)),
            obj_colour, CV_FILLED);
    // bottom
    cvRectangle(image8u3,
            cvPoint(round(tilt_bar_col_start),
                    round(tilt_median + tilt_distr_half_len + TICK_WIDTH)),
            cvPoint(round(tick_col_end),
                    round(tilt_median + tilt_distr_half_len)),
            obj_colour, CV_FILLED);

    // render VFOA
    string obj_name;
    CvScalar target_colour;
#ifdef __VFOA_MODULE_FOUND__
    distribution_t vfoa_distr =
            mModel.vfoa_model()->GetVfoaDistributionForId(tracker->id());
//    cout << "ID: " << tracker->id() << ", distribution: " << vfoa_distr << endl;
    float left_offset = main_roi.m_iFirstColumn;
    float max_proba = 0;
    BOOST_FOREACH(const prob_element_t& elt, vfoa_distr) {
        ostringstream obj_name_oss;
        CvScalar colour;
        switch (elt.target.type) {
         case UNFOCUSSED_VALUE:
             colour = CV_RGB(255, 255, 255);
             obj_name_oss << "Unfocused";
             break;
         case OBJECT_INFO:
             colour = VFOA_OBJECT_COLOURS[elt.target.id %
                                          VFOA_OBJECT_COLOURS.size()];
             obj_name_oss << mModel.vfoa_model()->GetObjectInfo(elt.target.id).name;
             break;
         case PERSON_INFO:
             colour = VFOA_TARGET_COLOURS[elt.target.id %
                                          VFOA_TARGET_COLOURS.size()];
             obj_name_oss << mModel.vfoa_model()->GetPersonInfo(elt.target.id).name;
             break;
         default:
             obj_name_oss << "Error";
             break;
        }
        float proba = elt.prob;
        if (max_proba < proba) {
            obj_name = obj_name_oss.str();
            target_colour = colour;
            max_proba = proba;
        }
        cvRectangle(image8u3,
            cvPoint(left_offset,
                    main_roi.m_iFirstRow + main_roi.m_iHeight +
                    VFOA_BAR_OFFSET),
            cvPoint(left_offset + proba * main_roi.m_iWidth,
                    main_roi.m_iFirstRow + main_roi.m_iHeight +
                    VFOA_BAR_OFFSET + VFOA_BAR_WIDTH),
            colour, CV_FILLED);
        left_offset += proba * main_roi.m_iWidth;
    }
#else
    VfoaDistribution vfoa_distr = mModel.vfoa_model()->
            compute_vfoa_distribution_Hellinger(tracker);

    float left_offset = main_roi.m_iFirstColumn;
    BOOST_FOREACH(const VfoaDistribution::element_type& elt,
            vfoa_distr.elements()) {
        CvScalar colour = objId2ColourMap[elt.first.mId];
        float proba = elt.second;
        cvRectangle(image8u3,
            cvPoint(left_offset,
                    main_roi.m_iFirstRow + main_roi.m_iHeight +
                    VFOA_BAR_OFFSET),
            cvPoint(left_offset + proba * main_roi.m_iWidth,
                    main_roi.m_iFirstRow + main_roi.m_iHeight +
                    VFOA_BAR_OFFSET + VFOA_BAR_WIDTH),
            colour, CV_FILLED);
        left_offset += proba * main_roi.m_iWidth;
    }

    vfoa_CognitiveVfoaModelObjectInfo obj_info = mode(vfoa_distr);
    target_colour = objId2ColourMap[obj_info.mId];
    obj_name = obj_info.mName;
#endif

    // render mode VFOA target name

    CvSize textSize;
    int baseline = 0;
    cvGetTextSize(obj_name.c_str(), &screenOutputFont,
            &textSize, &baseline);
    ip_RoiWindow roi_text;
    roi_text.m_iFirstColumn = min(max(main_roi.m_iFirstColumn, 0),
            image8u3->width - 1);
    roi_text.m_iFirstRow = min(max( static_cast<int>(round(
            main_roi.m_iFirstRow + main_roi.m_iHeight + VFOA_BAR_OFFSET +
            VFOA_BAR_WIDTH + 0.5 * OBJECT_NAME_OFFSET)), 0),
            image8u3->height - 1);
    roi_text.m_iWidth = min(textSize.width + OBJECT_NAME_OFFSET,
            image8u3->width - roi_text.m_iFirstColumn);
    roi_text.m_iHeight = min(
            static_cast<int>(round(textSize.height + OBJECT_NAME_OFFSET)),
            image8u3->height - roi_text.m_iFirstRow);
    blend_rect(image8u3, roi_text);
    cvPutText(image8u3, obj_name.c_str(),
            cvPoint(round(main_roi.m_iFirstColumn + 0.5 * OBJECT_NAME_OFFSET),
                    main_roi.m_iFirstRow + main_roi.m_iHeight +
                    VFOA_BAR_OFFSET + VFOA_BAR_WIDTH + OBJECT_NAME_OFFSET +
                    textSize.height
                    ),
            &black_font,
            CV_RGB(0, 0, 0));
    cvPutText(image8u3, obj_name.c_str(),
            cvPoint(round(main_roi.m_iFirstColumn + 0.5 * OBJECT_NAME_OFFSET),
                    main_roi.m_iFirstRow + main_roi.m_iHeight +
                    VFOA_BAR_OFFSET + VFOA_BAR_WIDTH + OBJECT_NAME_OFFSET +
                    textSize.height
                    ),
            &screenOutputFont,
            target_colour);

#ifdef SHOW_NOD_DETECTOR
    const NoddingDetector * nod_detector = mModel.nod_detector();
    if (nod_detector && nod_detector->detect_nod(*tracker)) {
        const string nod_str = "Nod";
        cvGetTextSize(nod_str.c_str(), &screenOutputFont, &textSize, &baseline);
        roi_text.m_iFirstColumn = min(max(
                roi_text.m_iFirstColumn - textSize.width, 0), image8u3->width - 1);
        roi_text.m_iWidth = min(textSize.width,
                image8u3->width - roi_text.m_iFirstColumn);
        roi_text.m_iHeight = min(
                static_cast<int>(round(textSize.height)),
                image8u3->height - roi_text.m_iFirstRow);
        roi_text.m_iFirstRow -= textSize.height;
        blend_rect(image8u3, roi_text);
        cvPutText(image8u3, nod_str.c_str(),
                cvPoint(roi_text.m_iFirstColumn, roi_text.m_iFirstRow +
                        roi_text.m_iHeight),
                &black_font,
                CV_RGB(0, 0, 0));
        cvPutText(image8u3, nod_str.c_str(),
                cvPoint(roi_text.m_iFirstColumn, roi_text.m_iFirstRow +
                        roi_text.m_iHeight),
                &screenOutputFont,
                obj_colour);
    }
#endif

}

/* static */
void MainWindow::blend_rect(IplImage * image8u3,
        const ImageProcessing::ip_RoiWindow& roi) {
    for (int i = roi.m_iFirstRow; i < roi.m_iFirstRow + roi.m_iHeight; ++i) {
        for (int j = roi.m_iFirstColumn; j < roi.m_iFirstColumn + roi.m_iWidth; ++j) {
            CV_IMAGE_ELEM(image8u3, uchar, i, 3 * j) =
                    CV_IMAGE_ELEM(image8u3, uchar, i, 3 * j) / 2 + 127;
            CV_IMAGE_ELEM(image8u3, uchar, i, 3 * j + 1) =
                    CV_IMAGE_ELEM(image8u3, uchar, i, 3 * j + 1) / 2 + 127;
            CV_IMAGE_ELEM(image8u3, uchar, i, 3 * j + 2) =
                    CV_IMAGE_ELEM(image8u3, uchar, i, 3 * j + 2) / 2 + 127;
        }
    }
}

/* static */ vector<CvScalar> initialize_vfoa_tracker_colours() {
    vector<CvScalar> colours(7);
    unsigned idx = 0;
    colours[idx++] = CV_RGB(  0,   0, 255);
    colours[idx++] = CV_RGB(  0, 127,   0);
    colours[idx++] = CV_RGB(255,   0,   0);
    colours[idx++] = CV_RGB(  0, 191, 191);
    colours[idx++] = CV_RGB(191,   0, 191);
    colours[idx++] = CV_RGB(191, 191,   0);
    colours[idx++] = CV_RGB( 64,  64,  64);
    return colours;
} // initialize_vfoa_tracker_colours

/* static */ vector<CvScalar> initialize_vfoa_object_colours() {
    vector<CvScalar> colours(7);
    unsigned idx = 0;
    colours[idx++] = CV_RGB(115, 115, 255);
    colours[idx++] = CV_RGB(103, 228, 103);
    colours[idx++] = CV_RGB(255, 115, 115);
    colours[idx++] = CV_RGB(103, 228, 228);
    colours[idx++] = CV_RGB(228, 103, 228);
    colours[idx++] = CV_RGB(228, 228, 103);
    colours[idx++] = CV_RGB(128, 128, 128);
    return colours;
} // initialize_vfoa_object_colours
