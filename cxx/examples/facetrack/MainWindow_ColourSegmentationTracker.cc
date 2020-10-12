// Copyright (c) 2011-2020 Idiap Research Institute
//
// MainWindow_ColourSegmentationTracker - class to visualise main model data
//                                        for colour segmentation-based tracking
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/version.hpp>                        // boost version
#include <boost/foreach.hpp>                        // foreach loop

#if BOOST_VERSION > 104700
#include <boost/random/gamma_distribution.hpp>      // gamma random samples
#include <boost/random/normal_distribution.hpp>     // normal random samples
#include <boost/random/student_t_distribution.hpp>  //
#include <boost/random/variate_generator.hpp>       // variate generator
#endif

#include <boost/lambda/lambda.hpp>                  // lambda expressions
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // parameters2roi converter
#include <numeric>                                  // for accumulate
#include <map>                                      // STL map
#include <iomanip>

#include <highgui.h>

// PROJECT INCLUDES
#include <opencvplus/cvp_helper_functions.h>        // for ypbpr2bgr
#include <image_processing/ip_Dense2DMotionProcessorInria.h>

// LOCAL INCLUDES
#include "MainWindow_ColourSegmentationTracker.h"   // declaration of this

using namespace std;
using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace BICV;

/////////////////////////// GLOBAL CONSTANTS /////////////////////////////////

static vector<CvScalar> initialize_vfoa_colours();
static vector<CvScalar> initialize_orientation_colours();
static vector<CvScalar> initialize_pim_colours();

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

static const int PALETTE_IMAGE_SIZE = 32;
static const int PALETTE_IMAGE_OFFSET_X = PALETTE_IMAGE_SIZE + 30;
static const int PALETTE_IMAGE_SPACING_Y = 5;
static const int PIM_IMAGE_SIZE = 32;
static const int PIM_IMAGE_OFFSET_X = PIM_IMAGE_SIZE +
        PALETTE_IMAGE_OFFSET_X + 5;
static const int PIM_IMAGE_SPACING_Y = 5;

static const bool SHOW_OBJECT_NUMBER = true;
static const bool SHOW_MEAN_NOT_MODE = true;

// Render motion vectors on the resulting image
//#define SHOW_MOTION
// Render motion properties on the resulting image
//#define SHOW_MOTION_PROPERTIES
// Render likelihood value for the rendered state on the resulting image
//#define SHOW_LIKELIHOOD
// Render pose probabilities
//#define SHOW_POSE_PROBABILITIES
// Render motion support using transparent colours
//#define SHOW_MOTION_SUPPORT
// Render all particles
//#define SHOW_ALL_PARTICLES
// Render pixel classes with different colours
#define SHOW_SKIN_COLOUR_CLASSIFICATION
// Render pixel classes with different colours
//#define SHOW_SKIN_COLOUR_PROBABILITY
// Show skin colour model palettes
//#define SHOW_PALETTES
// Show skin colour model PIMs
//#define SHOW_PIMS
// Render face rectangles
#define SHOW_DETECTED_FACES
// Render tracker performance in FPS
//#define SHOW_FPS

static const vector<CvScalar> VFOA_TARGET_COLOURS = initialize_vfoa_colours();
static const vector<CvScalar> ORIENTATION_COLOURS =
        initialize_orientation_colours();
static const vector<CvScalar> PIM_COLOURS = initialize_pim_colours();

/////////////////////////////// PUBLIC ///////////////////////////////////////

MainWindow_ColourSegmentationTracker::MainWindow_ColourSegmentationTracker(
        const MainModel_ColourSegmentationTracker& model) : mModel(model) {
    mImage8u3 = cvCreateImage(
            cvSize(model.data_provider()->image_buffer()->width,
                   model.data_provider()->image_buffer()->height),
                   IPL_DEPTH_8U, 3 );
    mImageBuf8u3 = cvCreateImage(
            cvSize(model.data_provider()->image_buffer()->width,
                   model.data_provider()->image_buffer()->height),
            IPL_DEPTH_8U, 3 );
    mRenderBuf8u3 = cvCreateImage(
            cvSize(model.data_provider()->image_buffer()->width,
                   model.data_provider()->image_buffer()->height),
            IPL_DEPTH_8U, 3 );
} // MainWindow_ColourSegmentationTracker

MainWindow_ColourSegmentationTracker::~MainWindow_ColourSegmentationTracker() {
    cvReleaseImage(&mImage8u3);
    cvReleaseImage(&mImageBuf8u3);
} // ~MainWindow_ColourSegmentationTracker

const IplImage * MainWindow_ColourSegmentationTracker::update() {

//    cout << "Main window update" << endl;

    CvFont font;
    cvInitFont(&font, CV_FONT_VECTOR0, 1.0, 1.0, 0.0, 1.0);

    // show original image
    IplImage * image = mModel.data_provider()->image();
//    IplImage * image = mModel.skin_mask_provider()->image();
//    cvCvtPlaneToPix(image, image, image, NULL, mImageBuf8u3);
    cvConvertScale(image, mImageBuf8u3, 1.0);
    cvCopy(mImageBuf8u3, mRenderBuf8u3);
//    cvPutText(mImageBuf8u3, "Original image", cvPoint( 50, 50),
//        &font, cvScalar(0, 0, 255));

    const list<bicv_ColourSegmentationTracker*>& trackers =
            mModel.trackers();

    // fill colour maps for VFOA targets
    map<unsigned, CvScalar> objId2Colour;
    for (unsigned i = 0; i < trackers.size(); ++i) {
        objId2Colour[i] = VFOA_TARGET_COLOURS[i % VFOA_TARGET_COLOURS.size()];
    }

#if (defined SHOW_MOTION_SUPPORT || defined SHOW_SKIN_COLOUR_LIKELIHOOD)
    const float tracker_coeff = 1.0f / (trackers.size() + 1);
#endif

    unsigned idx = 0;
    BOOST_FOREACH(bicv_ColourSegmentationTracker * tracker, trackers) {
//        cout << "Start render tracker " << tracker << ", time " <<
//                boost::posix_time::microsec_clock::local_time() << endl << flush;

#ifdef SHOW_SKIN_COLOUR_CLASSIFICATION
        show_tracker_colour_classification(mImageBuf8u3, mRenderBuf8u3, tracker);
#endif

#ifdef SHOW_SKIN_COLOUR_PROBABILITY
        show_tracker_colour_probability(tracker, mRenderBuf8u3, true, true);
#endif


        CvScalar colour = VFOA_TARGET_COLOURS[(idx++) % VFOA_TARGET_COLOURS.size()];
        render_tracker(tracker, image, mRenderBuf8u3, colour, idx);
//        cout << "End render tracker " << tracker << ", time " <<
//                boost::posix_time::microsec_clock::local_time() << endl << flush;


#ifdef SHOW_MOTION_SUPPORT
        show_motion_support(tracker, mRenderBuf8u3, colour, tracker_coeff);
#endif
    }

#ifdef SHOW_DETECTED_FACES
    show_detected_faces(mRenderBuf8u3);
#endif

#ifdef SHOW_FPS
    show_fps(mRenderBuf8u3);
#endif

    cvResize( mRenderBuf8u3, mImage8u3, CV_INTER_LINEAR );

    return mImage8u3;
} // update

void MainWindow_ColourSegmentationTracker::show_motion_support(
        BICV::bicv_ColourSegmentationTracker * tracker,
        IplImage * image8u3,
        const CvScalar & obj_colour, float coeff) {

    bicv_TrackerState propagated_state = (SHOW_MEAN_NOT_MODE ?
            mean(tracker->particle_distribution()) :
            mode(tracker->particle_distribution()));
    ip_RoiWindow tracker_roi = HpParams2RoiConverter::params2roi(
            propagated_state.m_ParamsCur);
    ip_RoiWindow roi = tracker_roi;

    static const double support_scale = 1.2;
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

    ip_Dense2DMotionProcessorInria * motion_proc =
            dynamic_cast<ip_Dense2DMotionProcessorInria*>(
                    mModel.dense_motion_processor());
    motion_proc->reestimate4roi(roi); // recomputes results for ROI

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


void MainWindow_ColourSegmentationTracker::show_detected_faces(
        IplImage * image8u3) {
    const list<cvp_FaceDescriptor>& faces = mModel.faces_cache();
    BOOST_FOREACH(cvp_FaceDescriptor fd, faces) {
        cvRectangle(image8u3, cvPoint(fd.m_FaceRegion.x, fd.m_FaceRegion.y),
            cvPoint(fd.m_FaceRegion.x + fd.m_FaceRegion.width,
                    fd.m_FaceRegion.y + fd.m_FaceRegion.height),
                    cvScalar(255, 0, 255),
            2);
    }
} // show_detected_faces

void MainWindow_ColourSegmentationTracker::show_fps(IplImage * image8u3) {
    CvFont screenOutputFont;
    cvInitFont(&screenOutputFont, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0, 2.);

    float fps = mModel.performance_fps();
    ostringstream oss;
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
} // show_fps

void MainWindow_ColourSegmentationTracker::render_tracker(
        bicv_ColourSegmentationTracker * tracker,
        IplImage * source_image8u3,
        IplImage * image8u3,
        const CvScalar & obj_colour, unsigned idx) {

    CvFont screenOutputFont;
    cvInitFont(&screenOutputFont, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0, 2.);
    CvFont black_font;
    cvInitFont(&black_font, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0.0, 3.0);
    CvFont system_font;
    cvInitFont(&system_font, CV_FONT_HERSHEY_PLAIN, .8, .8, 0.0, 1.0);

    // render mode
    bicv_TrackerState shown_state = (SHOW_MEAN_NOT_MODE ?
            mean(tracker->particle_distribution()) :
            mode(tracker->particle_distribution()));
    ip_RoiWindow main_roi = HpParams2RoiConverter::params2roi(
            shown_state.m_ParamsCur);
    cvRectangle(image8u3, cvPoint(main_roi.m_iFirstColumn, main_roi.m_iFirstRow),
        cvPoint(main_roi.m_iFirstColumn + main_roi.m_iWidth,
                main_roi.m_iFirstRow + main_roi.m_iHeight), obj_colour, 2);

//    FaceColorModel::FaceColorModel * colour_model = tracker->colour_model();
//    CvRect face_roi = colour_model->fcm_roi2face_roi(
//            ip_RoiWindow::to_CvRect(main_roi));
//    cvRectangle(image8u3, cvPoint(face_roi.x, face_roi.y),
//        cvPoint(face_roi.x + face_roi.width,
//                face_roi.y + face_roi.height), obj_colour, 2);

#ifdef SHOW_ALL_PARTICLES
    typedef BayesFilter::bf_DiscreteDistribution<bicv_TrackerState>
        ParticleDistribution;
    const ParticleDistribution& pdist = tracker->particle_distribution();
    ip_RoiWindow particle_roi;

    BOOST_FOREACH(const ParticleDistribution::element_type& elt,
            pdist.elements()) {
        particle_roi = HpParams2RoiConverter::params2roi(
                elt.first.m_ParamsCur);
        CvScalar colour = VFOA_TARGET_COLOURS[elt.first.m_Label %
                                              VFOA_TARGET_COLOURS.size()];
        cvRectangle(image8u3, cvPoint(particle_roi.m_iFirstColumn,
                particle_roi.m_iFirstRow),
            cvPoint(particle_roi.m_iFirstColumn + main_roi.m_iWidth - 1,
                    particle_roi.m_iFirstRow + main_roi.m_iHeight - 1),
                    colour, 1);
    }
#endif

    // render motion
#ifdef SHOW_MOTION
    ImageProcessing::ip_Dense2DMotionProcessor * motion_proc =
            mModel.dense_motion_processor();
    ip_RoiWindow roi = HpParams2RoiConverter::fcm_roi2face_roi(main_roi);
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
        const bicv_ColourSegmentationTracker::Likelihood * lhood_model =
                tracker->likelihood();
        float lhood = lhood_model->evaluate(shown_state, source_image8u3);
        oss.str("");
        oss << "L " << setw(num_fld_width) << lhood;
        motion_prop_str = oss.str();
        cvPutText(image8u3, motion_prop_str.c_str(),
                cvPoint(round(roi_text.m_iFirstColumn),
                        round(sys_offset + textSize.height)),
                &system_font,
                CV_RGB(255, 0, 0));
        sys_offset += textSize.height;
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

        oss.str("");
        oss << "div " << showpos << setw(num_fld_width) << parameters[2];
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
        ostringstream oss;
        oss << tracker->id();
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

#ifdef SHOW_POSE_PROBABILITIES
    show_pose_probabilities(tracker, image8u3);
#endif

#ifdef SHOW_PALETTES
    show_palettes(tracker, image8u3);
#endif
#ifdef SHOW_PIMS
    show_pims(tracker, image8u3);
#endif

} // render_tracker

void
MainWindow_ColourSegmentationTracker::show_tracker_colour_classification(
        IplImage * src_image8u3, IplImage * dst_image8u3,
        bicv_ColourSegmentationTracker * tracker) {

    bicv_TrackerState shown_state = (SHOW_MEAN_NOT_MODE ?
            mean(tracker->particle_distribution()) :
            mode(tracker->particle_distribution()));
    ip_RoiWindow main_roi = HpParams2RoiConverter::params2roi(
            shown_state.m_ParamsCur);
    ip_RoiWindow global_roi(0, 0, src_image8u3->width, src_image8u3->height);

    FaceColorModel::FaceColorModel * colour_model = tracker->colour_model();
    ip_RoiWindow v_roi_raw = ip_RoiWindow::from_CvRect(
        colour_model->face_roi2fcm_roi(ip_RoiWindow::to_CvRect(main_roi)));
    ip_RoiWindow v_roi;
    if (!intersect(global_roi, v_roi_raw, v_roi)) {
        return;
    }
    colour_model->cache_probability_maps(src_image8u3,
            ip_RoiWindow::to_CvRect(v_roi));
    const FaceColorModel::FaceColorModel::PimFeatureType * proba_maps =
            colour_model->cached_probability_maps();

    typedef FaceColorModel::FaceColorModel::PimFeatureType::value_type PType;

    using namespace FaceColorModel;

    vector<CvScalar> class_colours(FCM_NUM_CHANNELS);
    class_colours[FCM_CHANNEL_SKIN] = CV_RGB(255, 255,   0);
    class_colours[FCM_CHANNEL_HAIR] = CV_RGB(255,   0,   0);
    class_colours[FCM_CHANNEL_CLOTHES] = CV_RGB(  0, 255,   0);
    class_colours[FCM_CHANNEL_BACKGROUND] = CV_RGB(  0,   0,255);

    const CvScalar colour_default = CV_RGB( 64,  64,  64);
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

} // show_tracker_colour_classification

void
MainWindow_ColourSegmentationTracker::show_tracker_colour_probability(
        bicv_ColourSegmentationTracker * tracker, IplImage * image8u3,
        bool isPrior, bool isMap) {

    bicv_TrackerState shown_state = (SHOW_MEAN_NOT_MODE ?
            mean(tracker->particle_distribution()) :
            mode(tracker->particle_distribution()));
    ip_RoiWindow main_roi = HpParams2RoiConverter::params2roi(
            shown_state.m_ParamsCur);
    IplImage * roi_image8u3 = cvCreateImage(cvSize(main_roi.m_iWidth,
            main_roi.m_iHeight), IPL_DEPTH_8U, 3 );
    cvZero(roi_image8u3);

    // define warp matrix
    CvMat * warp_mat = cvCreateMat(2, 3, CV_32FC1);
    const float alpha = static_cast<float>(main_roi.m_iWidth) / main_roi.m_iWidth;
    const float beta = static_cast<float>(main_roi.m_iHeight) / main_roi.m_iHeight;
    CV_MAT_ELEM(*warp_mat, float, 0, 0) = alpha;
    CV_MAT_ELEM(*warp_mat, float, 0, 1) = 0;
    CV_MAT_ELEM(*warp_mat, float, 0, 2) = -alpha * main_roi.m_iFirstColumn;
    CV_MAT_ELEM(*warp_mat, float, 1, 0) = 0;
    CV_MAT_ELEM(*warp_mat, float, 1, 1) = beta;
    CV_MAT_ELEM(*warp_mat, float, 1, 2) = -beta * main_roi.m_iFirstRow;

    CvRect cvRoi = cvRect(main_roi.m_iFirstColumn, main_roi.m_iFirstRow,
            main_roi.m_iWidth, main_roi.m_iHeight);

    // warp source image to a temporary image buffer
    try {
//        static int counter = 1;
//        stringstream oss;
//        oss << "image" << setw(10) << setfill('0') << counter++ << ".jpg";
//        cvSetImageROI(image8u3, cvRoi);
        cvWarpAffine(image8u3, roi_image8u3, warp_mat);
//        cvSaveImage(oss.str().c_str(), roi_image8u3);
//        cvResize(image8u3, roi_image8u3, CV_INTER_LINEAR);
//        cvResetImageROI(image8u3);
    } catch (...) {
        cout << "Render tracker colour probability: error while extracting"
                "image patch " << main_roi << endl << flush;
    }

    cvReleaseMat(&warp_mat);

    FaceColorModel::FaceColorModel * colour_model = tracker->colour_model();
    const int LOGLHOOD_MAP_STEP = main_roi.m_iWidth * main_roi.m_iHeight;
    vector<FaceColorModel::fcm_real> loglhood_map(
            FaceColorModel::FCM_NUM_CHANNELS * LOGLHOOD_MAP_STEP, 0);

    // fill in probabilities
//    if (isPrior) {
//        const FaceColorModel::fcm_real * prior_probas = colour_model->get_class_posterior();
//
//        for (int row = 0; row < main_roi.m_iHeight; ++row) {
//            for (int col = 0; col < main_roi.m_iWidth; ++col) {
//            }
//        }
//
//        memcpy(&loglhood_map[0], prior_probas, FaceColorModel::FCM_NUM_CHANNELS
//                * LOGLHOOD_MAP_STEP * sizeof(FaceColorModel::fcm_real));
//    } else {
//        colour_model->channel_log_likelihood(roi_image8u3, &loglhood_map[0]);
//    }

    cvReleaseImage(&roi_image8u3);

    using namespace FaceColorModel;

    vector<CvScalar> class_colours(FCM_NUM_CHANNELS);
    class_colours[FCM_CHANNEL_SKIN] = CV_RGB(255, 255,   0);
    class_colours[FCM_CHANNEL_HAIR] = CV_RGB(255,   0,   0);
    class_colours[FCM_CHANNEL_CLOTHES] = CV_RGB(  0, 255,   0);
    class_colours[FCM_CHANNEL_BACKGROUND] = CV_RGB(  0,   0, 255);

    CvScalar colour_default = CV_RGB( 64,  64,  64);
    //CvScalar colour;

    vector<fcm_real> class_log_lhoods(FCM_NUM_CHANNELS);
    fcm_real sum;

    for (int row = 0; row < main_roi.m_iHeight; ++row) {
        for (int col = 0; col < main_roi.m_iWidth; ++col) {

            if ((main_roi.m_iFirstRow + row < 0) ||
                (main_roi.m_iFirstColumn + col < 0) ||
                (main_roi.m_iFirstRow + row >= image8u3->height) ||
                (main_roi.m_iFirstColumn + col >= image8u3->width)) {
                continue;
            }
            sum = (class_log_lhoods[FCM_CHANNEL_SKIN] = exp(loglhood_map[
                    FCM_CHANNEL_SKIN * LOGLHOOD_MAP_STEP +
                    row * main_roi.m_iWidth + col]));
            sum += (class_log_lhoods[FCM_CHANNEL_HAIR] = exp(loglhood_map[
                    FCM_CHANNEL_HAIR * LOGLHOOD_MAP_STEP +
                    row * main_roi.m_iWidth + col]));
            sum += (class_log_lhoods[FCM_CHANNEL_CLOTHES] = exp(loglhood_map[
                    FCM_CHANNEL_CLOTHES * LOGLHOOD_MAP_STEP +
                    row * main_roi.m_iWidth + col]));
            sum += (class_log_lhoods[FCM_CHANNEL_BACKGROUND] = exp(loglhood_map[
                    FCM_CHANNEL_BACKGROUND * LOGLHOOD_MAP_STEP +
                    row * main_roi.m_iWidth + col]));

            class_log_lhoods[FCM_CHANNEL_SKIN] /= sum;
            class_log_lhoods[FCM_CHANNEL_HAIR] /= sum;
            class_log_lhoods[FCM_CHANNEL_CLOTHES] /= sum;
            class_log_lhoods[FCM_CHANNEL_BACKGROUND] /= sum;

            CV_IMAGE_ELEM(image8u3, uchar, main_roi.m_iFirstRow + row,
                    3 * (main_roi.m_iFirstColumn + col)) =
                    class_log_lhoods[FCM_CHANNEL_SKIN] *
                    class_colours[FCM_CHANNEL_SKIN].val[0] +
                    class_log_lhoods[FCM_CHANNEL_HAIR] *
                    class_colours[FCM_CHANNEL_HAIR].val[0] +
                    class_log_lhoods[FCM_CHANNEL_CLOTHES] *
                    class_colours[FCM_CHANNEL_CLOTHES].val[0] +
                    class_log_lhoods[FCM_CHANNEL_BACKGROUND] *
                    class_colours[FCM_CHANNEL_BACKGROUND].val[0];
            CV_IMAGE_ELEM(image8u3, uchar, main_roi.m_iFirstRow + row,
                    3 * (main_roi.m_iFirstColumn + col) + 1) =
                    class_log_lhoods[FCM_CHANNEL_SKIN] *
                    class_colours[FCM_CHANNEL_SKIN].val[1] +
                    class_log_lhoods[FCM_CHANNEL_HAIR] *
                    class_colours[FCM_CHANNEL_HAIR].val[1] +
                    class_log_lhoods[FCM_CHANNEL_CLOTHES] *
                    class_colours[FCM_CHANNEL_CLOTHES].val[1] +
                    class_log_lhoods[FCM_CHANNEL_BACKGROUND] *
                    class_colours[FCM_CHANNEL_BACKGROUND].val[1];
            CV_IMAGE_ELEM(image8u3, uchar, main_roi.m_iFirstRow + row,
                    3 * (main_roi.m_iFirstColumn + col) + 2) =
                    class_log_lhoods[FCM_CHANNEL_SKIN] *
                    class_colours[FCM_CHANNEL_SKIN].val[2] +
                    class_log_lhoods[FCM_CHANNEL_HAIR] *
                    class_colours[FCM_CHANNEL_HAIR].val[2] +
                    class_log_lhoods[FCM_CHANNEL_CLOTHES] *
                    class_colours[FCM_CHANNEL_CLOTHES].val[2] +
                    class_log_lhoods[FCM_CHANNEL_BACKGROUND] *
                    class_colours[FCM_CHANNEL_BACKGROUND].val[2];
        }
    }


} // show_tracker_colour_probability

void MainWindow_ColourSegmentationTracker::show_pose_probabilities(
        BICV::bicv_ColourSegmentationTracker * tracker,
        IplImage * image8u3) {

    FaceColorModel::FaceColorModel * colour_model = tracker->colour_model();
    const unsigned num_models = colour_model->pim_models_count();

    typedef BICV::bicv_ColourSegmentationTracker::ParticleDistribution
            ParticleDistribution;
    const ParticleDistribution& particle_distr =
            tracker->particle_distribution();

    bicv_TrackerState shown_state = (SHOW_MEAN_NOT_MODE ?
            mean(particle_distr) : mode(particle_distr));
    ip_RoiWindow main_roi = HpParams2RoiConverter::params2roi(
            shown_state.m_ParamsCur);

    // evaluate distribution for poses
    vector<float> pose_distribution(num_models, 0);
    BOOST_FOREACH(const ParticleDistribution::element_type& weighted_particle,
            particle_distr.elements()) {
        pose_distribution[weighted_particle.first.m_ParamsCur.m_PoseIndex] +=
                weighted_particle.second;
    }
    float pose_distribution_sum = accumulate(
            pose_distribution.begin(), pose_distribution.end(), 0.0f);
    if (pose_distribution_sum < numeric_limits<float>::epsilon()) {
        // avoid division by zero
        pose_distribution_sum = 1.0f;
    }

    static const int POSE_PROBABILITIES_STRIPE_THICKNESS_PX = 5;
    static const int POSE_PROBABILITIES_STRIPE_OFFSET_PX = 25;

    // render stripes
    CvPoint upper_left = cvPoint(main_roi.m_iFirstColumn,
            main_roi.m_iFirstRow + main_roi.m_iHeight);
    CvPoint lower_right = cvPoint(main_roi.m_iFirstColumn + main_roi.m_iWidth,
            main_roi.m_iFirstRow + main_roi.m_iHeight +
            POSE_PROBABILITIES_STRIPE_THICKNESS_PX);
    int offset_x = main_roi.m_iFirstColumn;
    int offset_y = main_roi.m_iFirstRow + main_roi.m_iHeight +
            POSE_PROBABILITIES_STRIPE_OFFSET_PX;
    int offset;
    float pose_proba;
    for (unsigned idx = 0; idx < num_models; ++idx) {
        pose_proba = pose_distribution[idx] / pose_distribution_sum;
        offset = pose_proba * main_roi.m_iWidth;
        upper_left = cvPoint(offset_x, offset_y);
        lower_right = cvPoint(offset_x + offset,
            offset_y + POSE_PROBABILITIES_STRIPE_THICKNESS_PX);
        cvRectangle(image8u3, upper_left, lower_right,
            ORIENTATION_COLOURS[idx % ORIENTATION_COLOURS.size()], CV_FILLED);
        offset_x += offset;
    }

} // show_pose_probabilities

void MainWindow_ColourSegmentationTracker::show_palettes(
        BICV::bicv_ColourSegmentationTracker * tracker, IplImage * image8u3) {

        bicv_TrackerState shown_state = (SHOW_MEAN_NOT_MODE ?
            mean(tracker->particle_distribution()) :
            mode(tracker->particle_distribution()));
        ip_RoiWindow main_roi = HpParams2RoiConverter::params2roi(
            shown_state.m_ParamsCur);

        const cvp_PaletteBundle * palette_bundle = tracker->colour_model()->
                current_palette_bundle();
        unsigned num_palettes = palette_bundle->size();
        for (unsigned idx = 0; idx < num_palettes; ++idx) {
            const cvp_Palette * palette = palette_bundle->palette(idx);
            CvRect palette_rect = cvRect(
                main_roi.m_iFirstColumn - PALETTE_IMAGE_OFFSET_X,
                main_roi.m_iFirstRow +
                idx * (PALETTE_IMAGE_SIZE + PALETTE_IMAGE_SPACING_Y),
                PALETTE_IMAGE_SIZE, PALETTE_IMAGE_SIZE);
            if (palette_rect.x + palette_rect.width <= 0) {
                return;
            } else if (palette_rect.x < 0) {
                palette_rect.width += palette_rect.x;
                palette_rect.x = 0;
            }
            cvSetImageROI(image8u3, palette_rect);
            render_palette(palette, image8u3);
            cvResetImageROI(image8u3);
        }

} // show_palettes

void MainWindow_ColourSegmentationTracker::show_pims(
        BICV::bicv_ColourSegmentationTracker * tracker, IplImage * image8u3) {

    bicv_TrackerState shown_state = (SHOW_MEAN_NOT_MODE ?
        mean(tracker->particle_distribution()) :
        mode(tracker->particle_distribution()));
    ip_RoiWindow main_roi = HpParams2RoiConverter::params2roi(
        shown_state.m_ParamsCur);

    FaceColorModel::FaceColorModel * colour_model = tracker->colour_model();
    const unsigned num_pims = colour_model->pim_models_count();
    for (unsigned idx = 0; idx < num_pims; ++idx) {
        const cvp_PimModel * pim_model = colour_model->current_pim(idx);
        CvRect pim_rect = cvRect(
            main_roi.m_iFirstColumn - PIM_IMAGE_OFFSET_X,
            main_roi.m_iFirstRow + idx * (PIM_IMAGE_SIZE + PIM_IMAGE_SPACING_Y),
            PIM_IMAGE_SIZE, PIM_IMAGE_SIZE);
        if (pim_rect.x + pim_rect.width <= 0) {
            return;
        } else if (pim_rect.x < 0) {
            pim_rect.width += pim_rect.x;
            pim_rect.x = 0;
        }
        cvSetImageROI(image8u3, pim_rect);
        render_pim(pim_model, image8u3);
        cvResetImageROI(image8u3);
    }
} // show_pims

void MainWindow_ColourSegmentationTracker::render_palette(
        const OpenCvPlus::cvp_Palette* palette, IplImage * output_image) {
    switch(palette->type()) {
    case cvp_Palette::FCM_CONTINUOUS:
        render_continuous_palette(palette, output_image);
        break;
    case cvp_Palette::FCM_DISCRETE:
        render_discrete_palette(palette, output_image);
        break;
    }
} // render_palette

void MainWindow_ColourSegmentationTracker::render_discrete_palette(
        const OpenCvPlus::cvp_Palette* palette, IplImage * output_image) {

    const OpenCvPlus::cvp_Palette::Data& data = palette->data();
    DiscreteColorPrecompute* pPrecompute =
            (DiscreteColorPrecompute*)palette->precomputed_data();

    render_color_histogram(data, pPrecompute->binsNumPerDimension,
            output_image);
} // render_discrete_palette

void MainWindow_ColourSegmentationTracker::render_continuous_palette(
        const OpenCvPlus::cvp_Palette* palette, IplImage * output_image) {

#if BOOST_VERSION > 104700

        const OpenCvPlus::cvp_Palette::Data& data = palette->data();

        static const unsigned NUM_COLOUR_BINS = 8;
        static const unsigned NUM_SAMPLES = 2000;
        vector<double> distribution(NUM_COLOUR_BINS * NUM_COLOUR_BINS *
                NUM_COLOUR_BINS, 0);

        boost::mt19937 rng;

//        cout << "Continuous palette samples" << endl;
//        ostream_iterator<double> ostr_it(cout, ", ");
//        copy(data.begin(), data.end(), ostr_it);
//        cout << endl;

        CvScalar ypbpr_sample, bgr_sample;
        double eta, tau, alpha, beta, lambda_sample, mu_sample, col_sample;
        double sigma;
        int colour_index = 0;
        unsigned num_samples_left = NUM_SAMPLES;
        while (num_samples_left) {
            // generate YPbPr sample
            for (unsigned i = 0; i < 3; ++i) {
                eta = data[i * 4];
                tau = data[i * 4 + 1];
                alpha = data[i * 4 + 2];
                beta  = data[i * 4 + 3];
                sigma = sqrt(beta * (tau + 1) / (alpha * tau));

                boost::random::student_t_distribution<> student_distr(2 * alpha);
                boost::random::variate_generator<boost::mt19937&,
                    boost::random::student_t_distribution<> > var_student(
                            rng, student_distr);
                col_sample = var_student() * sigma + eta;

/*
                boost::random::gamma_distribution<> gamma_distr(alpha);
                boost::random::variate_generator<boost::mt19937&,
                    boost::random::gamma_distribution<> > var_gamma(
                            rng, gamma_distr);
                lambda_sample = var_gamma() / beta;

                boost::random::normal_distribution<> normal_distr(eta,
                        1.0 / sqrt(tau * lambda_sample));
                boost::random::variate_generator<boost::mt19937&,
                    boost::random::normal_distribution<> > var_normal(
                            rng, normal_distr);
                mu_sample = var_normal();

                boost::random::normal_distribution<> normal_col_distr(
                        mu_sample, 1.0 / sqrt(lambda_sample));
                boost::random::variate_generator<boost::mt19937&,
                    boost::random::normal_distribution<> > var_col_normal(
                            rng, normal_col_distr);
                col_sample = var_col_normal();
*/
                ypbpr_sample.val[i] = col_sample;
            }
            // convert YPbPr sample to BGR sample
            bgr_sample = ypbpr2bgr(ypbpr_sample);
            bool invalid_flag = false;
            for (unsigned i = 0; i < 3; ++i) {
                if ((bgr_sample.val[i] < 0) || (bgr_sample.val[i] > 255)) {
                    invalid_flag = true;
                    break;
                }
            }
            if (invalid_flag) {
                continue;
            }
//            cout << '(' << bgr_sample.val[0] << " "
//                        << bgr_sample.val[1] << " "
//                        << bgr_sample.val[2] << ") " << flush;
            // quantize sample and fill in the distribution vector
            colour_index = FaceColorModel::FaceColorModel::bgr_to_histogram(
                    bgr_sample, NUM_COLOUR_BINS);
//            cout << colour_index << " " << flush;
            distribution[colour_index] += 1;

            --num_samples_left;
        }
        render_color_histogram(distribution, NUM_COLOUR_BINS, output_image);
#endif // BOOST_VERSION > 104700
} // render_continuous_palette

void MainWindow_ColourSegmentationTracker::render_color_histogram(
        const vector<double>& histogram, unsigned bins_num,
        IplImage * output_image) {

    static const unsigned HIST_IMAGE_SIZE = 64;
    IplImage * hist_image_8u3 = cvCreateImage(
            cvSize(HIST_IMAGE_SIZE, HIST_IMAGE_SIZE), IPL_DEPTH_8U, 3);

    std::vector<double> hist(histogram);
    double hist_sum = accumulate(hist.begin(), hist.end(), 0.0);
    transform(hist.begin(), hist.end(), hist.begin(),
            boost::lambda::_1 / hist_sum);

    unsigned image_index = 0;
    const unsigned image_index_max = HIST_IMAGE_SIZE * HIST_IMAGE_SIZE;

//    cout << "Render colour histogram" << endl;

    vector<double>::iterator it;
    vector<double>::iterator it_begin = hist.begin();
    unsigned num_pixels;
    while (image_index < image_index_max) {
        it = max_element(hist.begin(), hist.end());
        num_pixels = max(static_cast<unsigned>((*it) * image_index_max), 1u);
//        cout << "value " << *it << ", num_pixels " << num_pixels << endl;
        CvScalar colour = FaceColorModel::FaceColorModel::histogram_to_bgr(
                distance(it_begin, it), bins_num);
//        cout << "index " << distance(it_begin, it) << ", colour "
//             << colour.val[0] << ", " << colour.val[1] << ", " << colour.val[2] << endl;
        for (unsigned i = 0; (i < num_pixels) &&
            (image_index < image_index_max); ++i) {
            for (unsigned j = 0; j < 3; ++j) {
                CV_IMAGE_ELEM(hist_image_8u3, uchar,
                        image_index / HIST_IMAGE_SIZE,
                        (image_index % HIST_IMAGE_SIZE) * 3 + j) =
                        static_cast<uchar>(colour.val[j]);
            }
            ++image_index;
        }
        *it = 0;
    }
    cvResize(hist_image_8u3, output_image);
    cvReleaseImage(&hist_image_8u3);

} // render_color_histogram

void MainWindow_ColourSegmentationTracker::render_pim(
        const OpenCvPlus::cvp_PimModel* pim, IplImage * output_image) {

    typedef cvp_PimModel::PimFeatureType PimFeatureType;
    typedef PimFeatureType::value_type Real;

    const PimFeatureType& pim_data = pim->data();
    const unsigned width = pim_data.width();
    const unsigned height = pim_data.height();
    IplImage * pim_image_8u3 = cvCreateImage(cvSize(width, height),
            IPL_DEPTH_8U, 3);

    const unsigned num_channels = pim_data.channels();
    vector<Real> pim_vals(num_channels);
    const vector<Real>::iterator it_beg = pim_vals.begin();
    const vector<Real>::iterator it_end = pim_vals.end();
    vector<Real>::iterator it_max;
    CvScalar colour;

    for (unsigned row = 0; row < height; ++row) {
        for (unsigned col = 0; col < width; ++col) {
            for (unsigned idx = 0; idx < num_channels; ++idx) {
                pim_vals[idx] = CV_MAT_ELEM(*pim_data.map(idx), Real, row, col);
            }
            it_max = max_element(it_beg, pim_vals.end());
            colour = PIM_COLOURS[distance(it_beg, it_max)];
            for (unsigned j = 0; j < 3; ++j) {
                CV_IMAGE_ELEM(pim_image_8u3, uchar, row, col * 3 + j) =
                    static_cast<uchar>(colour.val[j]);
            }
        }
    }
    cvResize(pim_image_8u3, output_image);
    cvReleaseImage(&pim_image_8u3);

} // render_pim

/* static */
void MainWindow_ColourSegmentationTracker::blend_rect(IplImage * image8u3,
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
} // blend_rect

/* static */ vector<CvScalar> initialize_vfoa_colours() {
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
} // initialize_vfoa_colours

/* static */ vector<CvScalar> initialize_orientation_colours() {
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
} // initialize_orientation_colours

/* static */ vector<CvScalar> initialize_pim_colours() {
    vector<CvScalar> colours(5);
    unsigned idx = 0;
    colours[idx++] = CV_RGB(255, 255,   0);
    colours[idx++] = CV_RGB(255,   0,   0);
    colours[idx++] = CV_RGB(  0, 255,   0);
    colours[idx++] = CV_RGB(  0,   0, 255);
    colours[idx++] = CV_RGB(  0,   0,   0);
    return colours;
} // initialize_pim_colours
