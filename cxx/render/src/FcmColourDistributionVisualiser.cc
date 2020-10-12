/**
 * @file cxx/render/render/FcmColourDistributionVisualiser.cc
 * @date 19 June 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Visualisation for face colour model colour distributions
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>          // RNG
#include <boost/random.hpp>                           // taus88 RNG
//#include <boost/random/student_t_distribution.hpp>    // Student's t-distr
#include <boost/random/variate_generator.hpp>         // sampling distr
#include <ctime>                                      // time

// PROJECT INCLUDES
#include <opencvplus/cvp_IplDepthTraits.h>            // type to depth converter

// LOCAL INCLUDES
#include "render/FcmColourDistributionVisualiser.h"   // declaration of this

using namespace OpenCvPlus;
using namespace std;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const int DISTR_IMG_WIDTH = 64;
static const int DISTR_IMG_HEIGHT = 64;
static const int W_OFFSET = 4;
static const int H_OFFSET = 4;

static const unsigned NUM_SAMPLES_CONT_DISTR = 10000;

/////////////////////////////// PUBLIC ///////////////////////////////////////

namespace RenderUtils {

FcmColourDistributionVisualiser::FcmColourDistributionVisualiser(
        FaceColorModel::FaceColorModel * model, unsigned num_rows,
        unsigned num_cols) :
        m_FaceColourModel(model), m_NumCols(num_cols), m_NumRows(num_rows) {

    // initialise title font
    cvInitFont(&m_TitleFont, CV_FONT_HERSHEY_PLAIN, 1.5, 1., 0, 2.);
    int baseline = 0;
    CvSize font_size;
    cvGetTextSize("TdypQqgb", &m_TitleFont, &font_size, &baseline);
    m_TitleFontWidth = font_size.width;
    m_TitleFontHeight = font_size.height;

    // initialise buffers to render distributions ...
    m_DistrRenderBuffer = cvCreateImage(
            cvSize(DISTR_IMG_WIDTH, DISTR_IMG_HEIGHT), IPL_DEPTH_8U, 3);
    // ... and distributions with title
    const int width_dwt = DISTR_IMG_WIDTH + 2 * W_OFFSET;
    const int height_dwt = DISTR_IMG_HEIGHT + 2 * H_OFFSET +
            m_TitleFontHeight + 2 * H_OFFSET;
    m_DistrWTitleRenderBuffer = cvCreateImage(cvSize(width_dwt, height_dwt),
            IPL_DEPTH_8U, 3);

    // initialise buffer to store rendered distributions
    const int width = num_cols * width_dwt;
    const int height = num_rows * height_dwt;
    m_RenderBuffer = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

} // FcmColourDistributionVisualiser

FcmColourDistributionVisualiser::~FcmColourDistributionVisualiser() {
    // release buffer used to store rendered distributions
    cvReleaseImage(&m_RenderBuffer);
    // release buffer used to render distributions with title
    cvReleaseImage(&m_DistrWTitleRenderBuffer);
    // release buffer used to render distributions
    cvReleaseImage(&m_DistrRenderBuffer);
} // ~FcmColourDistributionVisualiser

IplImage *  FcmColourDistributionVisualiser::visualise() {

    const int width_dwt = DISTR_IMG_WIDTH + 2 * W_OFFSET;
    const int height_dwt = DISTR_IMG_HEIGHT + 2 * H_OFFSET +
            m_TitleFontHeight + 2 * H_OFFSET;

    int baseline = 0;
    CvSize font_size;

    unsigned i,j;

    for (unsigned channel = 0; channel < FaceColorModel::FCM_NUM_CHANNELS;
        ++channel) {

        j = channel / m_NumCols;
        i = channel % m_NumCols;
        if (j < m_NumRows) {
            // render distribution to m_DistrRenderBuffer
            render_distribution(channel, m_DistrRenderBuffer);

            // copy rendered distribution to m_DistrWTitleRenderBuffer
            cvZero(m_DistrWTitleRenderBuffer);
            CvRect roi = cvRect(W_OFFSET, m_TitleFontHeight + 2 * H_OFFSET,
                    DISTR_IMG_WIDTH, DISTR_IMG_HEIGHT);
            cvSetImageROI(m_DistrWTitleRenderBuffer, roi);
            cvCopy(m_DistrRenderBuffer, m_DistrWTitleRenderBuffer);
            cvResetImageROI(m_DistrWTitleRenderBuffer);

            // add title
            string title_str;
            switch(channel) {
            case FaceColorModel::FCM_CHANNEL_SKIN:
                title_str = "skin";
                break;
            case FaceColorModel::FCM_CHANNEL_HAIR:
                title_str = "hair";
                break;
            case FaceColorModel::FCM_CHANNEL_CLOTHES:
                title_str = "clothes";
                break;
            case FaceColorModel::FCM_CHANNEL_BACKGROUND:
                title_str = "bg";
                break;
            default:
                title_str = "unknown";
                break;
            }
            cvGetTextSize(title_str.c_str(), &m_TitleFont, &font_size, &baseline);
            m_TitleFontWidth = font_size.width;
            cvPutText(m_DistrWTitleRenderBuffer, title_str.c_str(),
                    cvPoint(static_cast<int>(round(
                        (width_dwt - m_TitleFontWidth) / 2)),
                        m_TitleFontHeight + H_OFFSET),
                    &m_TitleFont,
                    CV_RGB(255, 0, 0));

            // copy distribution with title to the resulting image
            CvRect roi_cell = cvRect(i * width_dwt, j * height_dwt,
                    width_dwt, height_dwt);
            cvSetImageROI(m_RenderBuffer, roi_cell);
            cvCopy(m_DistrWTitleRenderBuffer, m_RenderBuffer);
            cvResetImageROI(m_RenderBuffer);
        }
    }
    return m_RenderBuffer;
} // visualise

void
FcmColourDistributionVisualiser::render_distribution(
        int channel, IplImage * buffer) {

//    int distr_type = m_FaceColourModel->get_channel_type(channel);
//    switch(distr_type) {
//    case cvp_Palette::FCM_DISCRETE:
//        render_discrete_colour_distribution(m_FaceColourModel,
//            m_FaceColourModel->get_color_posterior(channel), buffer);
//        break;
//    case cvp_Palette::FCM_CONTINUOUS:
//        render_continuous_colour_distribution(m_FaceColourModel,
//                m_FaceColourModel->get_color_posterior(channel), buffer);
//        break;
//    }
} // render_distribution

/* static */ void
FcmColourDistributionVisualiser::visualise(
        FaceColorModel::FaceColorModel * model,
        IplImage * image, unsigned num_rows,
        unsigned num_cols) {

} // visualise

/////////////////////////////// PRIVATE //////////////////////////////////////

void ypbpr2rgb(const vector<FaceColorModel::fcm_real>& ypbpr,
    vector<FaceColorModel::fcm_real>& rgb) {

    const double a11 = 1.0, a12 = -0.000001218894189, a13 =  1.401999588657340;
    const double a21 = 1.0, a22 = -0.344135678165337, a23 = -0.714136155581812;
    const double a31 = 1.0, a32 =  1.772000066073816, a33 =  0.000000406298063;

    const FaceColorModel::fcm_real * pY  = &ypbpr[0];
    const FaceColorModel::fcm_real * pPb = &ypbpr[NUM_SAMPLES_CONT_DISTR];
    const FaceColorModel::fcm_real * pPr = &ypbpr[2 * NUM_SAMPLES_CONT_DISTR];
    FaceColorModel::fcm_real Y;
    FaceColorModel::fcm_real Pb;
    FaceColorModel::fcm_real Pr;

    FaceColorModel::fcm_real * pR = &rgb[0];
    FaceColorModel::fcm_real * pG = &rgb[NUM_SAMPLES_CONT_DISTR];
    FaceColorModel::fcm_real * pB = &rgb[2 * NUM_SAMPLES_CONT_DISTR];

    for (unsigned j = 0; j < NUM_SAMPLES_CONT_DISTR; ++j) {
        Y = *pY++;
        Pb = *pPb++ - 0.5;
        Pr = *pPr++ - 0.5;
        *pR++ = a11 * Y + a12 * Pb + a13 * Pr;
        *pG++ = a21 * Y + a22 * Pb + a23 * Pr;
        *pB++ = a31 * Y + a32 * Pb + a33 * Pr;
    }

    // correct to range [0, 1]
    pR = &rgb[0];
    pG = &rgb[NUM_SAMPLES_CONT_DISTR];
    pB = &rgb[2 * NUM_SAMPLES_CONT_DISTR];
    for (unsigned j = 0; j < NUM_SAMPLES_CONT_DISTR; ++j) {
        *pR = max(min(*pR, 1.0), 0.0); pR++;
        *pG = max(min(*pG, 1.0), 0.0); pG++;
        *pB = max(min(*pB, 1.0), 0.0); pB++;
    }
} // ypbpr2rgb

void rgb2bins(FaceColorModel::FaceColorModel * model,
        const vector<FaceColorModel::fcm_real>& rgb, vector<int>& bins) {
/*    const FaceColorModel::fcm_real * pR = &rgb[0];
    const FaceColorModel::fcm_real * pG = &rgb[NUM_SAMPLES_CONT_DISTR];
    const FaceColorModel::fcm_real * pB = &rgb[2 * NUM_SAMPLES_CONT_DISTR];
    FaceColorModel::fcm_real R;
    FaceColorModel::fcm_real G;
    FaceColorModel::fcm_real B;

    int* pBins = &bins[0];
    int rBin;
    int gBin;
    int bBin;
    int bin_val;

    for (unsigned j = 0; j < NUM_SAMPLES_CONT_DISTR; ++j) {
        R = *pR++;
        G = *pG++;
        B = *pB++;
        rBin = (static_cast<int>(R * 255) * model->get_histogram_bins_num()) >> 8;
        gBin = (static_cast<int>(G * 255) * model->get_histogram_bins_num()) >> 8;
        bBin = (static_cast<int>(B * 255) * model->get_histogram_bins_num()) >> 8;
        bin_val =
            rBin * model->get_histogram_bins_num() * model->get_histogram_bins_num() +    \
            gBin * model->get_histogram_bins_num() +           \
            bBin;
        if (bin_val >= model->get_histogram_size()) {
            cout << "R=" << R << ", G=" << G << ", B=" << B << ", bin=" << bin_val << endl;
            assert(bin_val < model->get_histogram_size());
        }
        *pBins++ = bin_val;
    }*/
} // rgb2bins

void bins2hist(const vector<int>& bins, vector<FaceColorModel::fcm_real>& hist) {
    const int* pBins = &bins[0];
    for (unsigned j = 0; j < NUM_SAMPLES_CONT_DISTR; ++j) {
        hist[*pBins++]++;
    }
} // bins2hist

void FcmColourDistributionVisualiser::render_continuous_colour_distribution(
        FaceColorModel::FaceColorModel * model,
        const FaceColorModel::fcm_real * parameters,
        IplImage * buffer) {

/*    typedef boost::random::student_t_distribution<FaceColorModel::fcm_real>
        student_dist_type;
    typedef boost::random::variate_generator<boost::mt19937&, student_dist_type>
        student_gen_type;

    static const unsigned NUM_BOOTSTRAP_ITER = 100;
    boost::taus88 rng_tau(static_cast<unsigned>(time(0)));
    for (unsigned i = 0; i < NUM_BOOTSTRAP_ITER; ++i) {
        rng_tau();
    }
    boost::mt19937 rng_mt(rng_tau());

    vector<FaceColorModel::fcm_real> samples_ypbpr (
            3 * NUM_SAMPLES_CONT_DISTR, 0);

    FaceColorModel::fcm_real eta;
    FaceColorModel::fcm_real tau;
    FaceColorModel::fcm_real alpha;
    FaceColorModel::fcm_real beta;

    FaceColorModel::fcm_real nu;
    FaceColorModel::fcm_real sigma;

    // sample YPbPr components
    cout << "sampling ..." << endl;
    FaceColorModel::fcm_real * samples_ptr = &samples_ypbpr[0];
    for (int i = 0; i < 3; ++i) {
        eta   = parameters[0+4*i];
        tau   = parameters[1+4*i];
        alpha = parameters[2+4*i];
        beta  = parameters[3+4*i];
        nu = 2 * alpha;
        sigma = sqrt((tau + 1) * beta / (tau * alpha));

        student_dist_type student_dist(nu);
        student_gen_type student_gen(rng_mt, student_dist);

        for (unsigned j = 0; j < NUM_SAMPLES_CONT_DISTR; ++j) {
            *samples_ptr++ = student_gen() * sigma + eta;
        }
    }

    // convert to RGB
    cout << "converting YPbPr to RGB ..." << endl;
    vector<FaceColorModel::fcm_real> samples_rgb (
            3 * NUM_SAMPLES_CONT_DISTR, 0);
    ypbpr2rgb(samples_ypbpr, samples_rgb);

    // convert to histogram bins
    cout << "converting RGB to bins ..." << endl;
    vector<int> samples_bins (NUM_SAMPLES_CONT_DISTR, 0);
    rgb2bins(model, samples_rgb, samples_bins);

    // convert to histogram
    cout << "computing bin counts ..." << endl;
    vector<FaceColorModel::fcm_real> samples_bincounts (
            model->get_histogram_size(), 0);
    int * pBins = &samples_bins[0];
    for (unsigned j = 0; j < NUM_SAMPLES_CONT_DISTR; ++j) {
        samples_bincounts[*pBins++]++;
    }

    cout << "rendering discrete distribution ..." << endl;
    render_discrete_colour_distribution(model, &samples_bincounts[0], buffer);*/

} // render_continuous_colour_distribution

void sort(const FaceColorModel::fcm_real * values, vector<int> & indices) {
    int * pIndices = &indices[0];
    // ID transposition
    for (int j = 0; j < indices.size(); ++j) {
        *pIndices++ = j;
    }

    // bubble sort - TODO: replace!
    int buf = 0;
    for (int j = 0; j < indices.size(); ++j) {
        for (int i = j; i < indices.size(); ++i) {
            if (values[indices[j]] < values[indices[i]]) {
                buf = indices[j];
                indices[j] = indices[i];
                indices[i] = buf;
            }
        }
    }
} // sort

void FcmColourDistributionVisualiser::render_discrete_colour_distribution(
        FaceColorModel::FaceColorModel * model,
        const FaceColorModel::fcm_real * parameters,
        IplImage * buffer) {

/*    const int height = buffer->height;
    const int width = buffer->width;
    const int size = height * width;

    const unsigned HISTOGRAM_SIZE = model->get_histogram_size();
    const unsigned HISTOGRAM_BINS = model->get_histogram_bins_num();

    vector<int> indices_sorted (HISTOGRAM_SIZE, 0);

    // sorted transposition
    sort(parameters, indices_sorted);

    int cur_length = 0;

    // visualise first N bins (colours)
    for (unsigned j = 0; (j < HISTOGRAM_SIZE) && (cur_length < size); ++j) {
        const int index = indices_sorted[j];
        const int r_bin = index / (HISTOGRAM_BINS * HISTOGRAM_BINS);
        const int r_col = r_bin * 16;
        const int g_bin = index / HISTOGRAM_BINS;
        const int g_col = g_bin * 16;
        const int b_bin = index % HISTOGRAM_BINS;
        const int b_col = b_bin * 16;
        const int length = size * parameters[index];
        cout << "(" << r_col << ", " << g_col << ", " << b_col << ") x " << length << endl;
        if (length <= 0) {
            break;
        }
        for (int i = 0; (i < length) && (cur_length < size); ++i) {
            cvSet2D(buffer, cur_length / width, cur_length % width, CV_RGB(r_col, g_col, b_col));
            ++cur_length;
        }
    }*/
} // render_discrete_colour_distribution

} // namespace RenderUtils
