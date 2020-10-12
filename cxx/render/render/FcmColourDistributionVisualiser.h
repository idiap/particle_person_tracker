/**
 * @file cxx/render/render/FcmColourDistributionVisualiser.h
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

#ifndef __FCMCOLOURDISTRIBUTIONVISUALISER_H__
#define __FCMCOLOURDISTRIBUTIONVISUALISER_H__

// PROJECT INCLUDES
#include <opencvplus/FaceColorModel.h>          // face colour model

namespace RenderUtils {

/**
 * Visualizes PIM feature on an image. Probabilities for different classes
 * are colour-coded, intensity of a colour representing a class corresponds
 * to the probability in PIM.
 * @param pim_feature PIM feature to visualize
 * @return Image with visualisation of the PIM feature
 */
template<typename T>
IplImage *
visualize_pim_feature_probabilities(
        const OpenCvPlus::cvp_PimFeature<T>& pim_feature) {

    using namespace FaceColorModel;

    std::vector<CvScalar> class_colours(FCM_NUM_CHANNELS);
    class_colours[FCM_CHANNEL_SKIN] = CV_RGB(255, 255,   0);
    class_colours[FCM_CHANNEL_HAIR] = CV_RGB(255,   0,   0);
    class_colours[FCM_CHANNEL_CLOTHES] = CV_RGB(  0, 255,   0);
    class_colours[FCM_CHANNEL_BACKGROUND] = CV_RGB(  0,   0,255);

    OpenCvPlus::cvp_PimFeature<T> norm_pim_feature(pim_feature);
    norm_pim_feature.normalise_channels();

    const unsigned width = norm_pim_feature.width();
    const unsigned height = norm_pim_feature.height();

    IplImage * image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvZero(image);

    for (unsigned channel = 0; channel < FCM_NUM_CHANNELS; ++channel) {

        const CvMat * pim_channel = norm_pim_feature.map(channel);
        const CvScalar & colour = class_colours[channel];

        for (unsigned row = 0; row < height; ++row) {
            for (unsigned col = 0; col < width; ++col) {
                const T pim_val = CV_MAT_ELEM(*pim_channel, T, row, col);
                for (unsigned i = 0; i < 3; ++i) {
                    CV_IMAGE_ELEM(image, unsigned char, row, col * 3 + i) +=
                        static_cast<unsigned char>(pim_val * colour.val[i]);
                }
            }
        }
    }

    return image;

} // visualize_pim_feature


/**
 * Visualizes PIM feature assignments on an image.
 * Pixel assignment to a class is represented by the colour of that class.
 * @param pim_feature PIM feature to visualize
 * @return Image with visualisation of PIM feature assignments
 */
template<typename T>
IplImage *
visualize_pim_feature_classification(
        const OpenCvPlus::cvp_PimFeature<T>& pim_feature) {

    using namespace FaceColorModel;

    std::vector<CvScalar> class_colours(FCM_NUM_CHANNELS);
    class_colours[FCM_CHANNEL_SKIN] = CV_RGB(255, 255,   0);
    class_colours[FCM_CHANNEL_HAIR] = CV_RGB(255,   0,   0);
    class_colours[FCM_CHANNEL_CLOTHES] = CV_RGB(  0, 255,   0);
    class_colours[FCM_CHANNEL_BACKGROUND] = CV_RGB(  0,   0,255);

    OpenCvPlus::cvp_PimFeature<T> norm_pim_feature(pim_feature);
    norm_pim_feature.normalise_channels();

    const unsigned width = norm_pim_feature.width();
    const unsigned height = norm_pim_feature.height();

    IplImage * image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvZero(image);

    std::vector<T> values(FCM_NUM_CHANNELS);

    for (unsigned row = 0; row < height; ++row) {
        for (unsigned col = 0; col < width; ++col) {

            fill(values.begin(), values.end(), 0.0f);

            for (unsigned channel = 0; channel < FCM_NUM_CHANNELS; ++channel) {
                const CvMat * pim_channel = norm_pim_feature.map(channel);
                const T pim_val = CV_MAT_ELEM(*pim_channel, T, row, col);
                values[channel] = pim_val;
            }

            typename std::vector<T>::iterator max_val_iter =
                std::max_element(values.begin(), values.end());

            const CvScalar & colour = class_colours[
                std::distance(values.begin(), max_val_iter)];
            for (unsigned i = 0; i < 3; ++i) {
                CV_IMAGE_ELEM(image, unsigned char, row, col * 3 + i) +=
                    static_cast<unsigned char>(colour.val[i]);
            }
        }
    }

    return image;

} // visualize_pim_feature


/**
 * Class to visualise colour distributions of FaceColourModel
 */
class FcmColourDistributionVisualiser {

public:

    /**
     * Constructor, initializes visualiser
     * @param model Face colour model to visualise
     * @param num_rows Number of rows in the layout
     * @param num_cols Number of columns in the layout
     */
    FcmColourDistributionVisualiser(FaceColorModel::FaceColorModel * model,
            unsigned num_rows = 1,
            unsigned num_cols = FaceColorModel::FCM_NUM_CHANNELS);

    /**
     * Destructor
     */
    ~FcmColourDistributionVisualiser();

    /**
     * Visualises colour models on the provided image using specified layout.
     * Layout consists of cells, each cell visualises one colour model.
     * @return Image with visualisation of colour models
     */
    IplImage * visualise();

    /**
     * Visualises colour models on the provided image using specified layout.
     * Layout consists of cells, each cell visualises one colour model.
     * @param image Image on which to visualise colour models
     * @param num_rows Number of rows in the layout
     * @param num_cols Number of columns in the layout
     */
    static void visualise(FaceColorModel::FaceColorModel * model,
            IplImage * image, unsigned num_rows = 1,
            unsigned num_cols = FaceColorModel::FCM_NUM_CHANNELS);

private:

    void render_distribution(int channel, IplImage * buffer);

    void render_continuous_colour_distribution(
            FaceColorModel::FaceColorModel * model,
            const FaceColorModel::fcm_real * parameters,
            IplImage * buffer);

    void render_discrete_colour_distribution(
            FaceColorModel::FaceColorModel * model,
            const FaceColorModel::fcm_real * parameters,
            IplImage * buffer);

    // Face colour model to visualise
    FaceColorModel::FaceColorModel * m_FaceColourModel;

    // title font
    CvFont m_TitleFont;
    int m_TitleFontWidth;
    int m_TitleFontHeight;

    // Layout specification: number of columns and rows in grid representation
    // of colour models
    unsigned m_NumCols;
    unsigned m_NumRows;

    // Rendering buffer used to visualise the colour model
    IplImage * m_RenderBuffer;
    // Rendering buffer used to visualise colour distribution
    IplImage * m_DistrRenderBuffer;
    // Rendering buffer used to visualise colour distribution with title
    IplImage * m_DistrWTitleRenderBuffer;

};

} // namespace RenderUtils

#endif /* __FCMCOLOURDISTRIBUTIONVISUALISER_H__ */
