/**
 * @file cxx/image_processing/image_processing/ip_Dense2DMotionProcessorInria.h
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

#ifndef __IP_DENSE2DMOTIONPROCESSORINRIA_H__
#define __IP_DENSE2DMOTIONPROCESSORINRIA_H__

#ifdef __CMOTION2D_FOUND__

// PROJECT INCLUDES
#include <cmotion2d/CMotion2DModel.h>             // 2D motion model
#include <cmotion2d/CMotion2DEstimator.h>         // 2D motion estimator
#include <cmotion2d/CMotion2DWarping.h>           // 2D motion warping
#include <cmotion2d/CMotion2DImage.h>             // 2D motion image
#include <image_processing/ip_Dense2DMotionProcessor.h>  // base class
#include <image_processing/ip_Dense2DMotionProcessorInriaConfig.h>

namespace ImageProcessing {

/// @brief Class to extract dense 2D motion vector fields from
///        two consecutive images based on CMotion2D software from INRIA
///
/// This class implements the general dense 2D motion processor interface
/// to extract dense 2D motion vector fields from two consecutive images
/// based on CMotion2D software from INRIA.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_Dense2DMotionProcessorInria : public ip_Dense2DMotionProcessor  {

    public:

    // LIFECYCLE

    /// Constructor to initialize dense 2D motion processor
    /// @param img_provider Sequential image provider
    ip_Dense2DMotionProcessorInria(ip_ImageProvider * img_provider,
        const ip_Dense2DMotionProcessorInriaConfig& config);

    /// Destructor, deallocates partial image providers
    virtual ~ip_Dense2DMotionProcessorInria();

    // OPERATIONS

    /// Overrides base class method. Reestimates the motion model for
    /// a given ROI. Uses motion model definition provided in configuration
    /// options. Stores results in internal fields this class, they can be
    /// accessed through getModel.
    /// @param roi Region of interest for which to estimate the motion
    virtual void reestimate4roi(const ip_RoiWindow& roi);

    /// Reestimates the motion model for a given ROI.
    /// Uses external motion model and estimator to compute and store results.
    /// @param roi Region of interest for which to estimate the motion
    void reestimate4roi(const ip_RoiWindow& roi, CMotion2DModel * model,
        CMotion2DEstimator * estimator);

    /// Access to motion model used by this processor.
    /// @return Internal motion model of this motion processor
    CMotion2DModel * getModel() {
        return &m_MotionParametricModel;
    }

    /// Access to motion estimator used by this processor.
    /// @return Internal motion estimator of this motion processor
    CMotion2DEstimator * getEstimator() {
        return &m_MotionEstimator;
    }

    const ImageProcessing::ip_Dense2DMotionProcessorInriaConfig&
    getConfig() const {
        return m_Config;
    }

    /// Applies provided configuration options to motion model and
    /// motion processor.
    /// @param config Configuration options for motion processor
    /// @param model Motion model used by the processor
    /// @param estimator Motion estimator used by this processor
    static void apply_config_options(
            const ip_Dense2DMotionProcessorInriaConfig& config,
            CMotion2DModel * model,
            CMotion2DEstimator * estimator);

    protected:

    // OPERATIONS

    /// Creates motion providers using source image information
    /// @param motx_provider Provider for X motion component
    /// @param moty_provider Provider for Y motion component
    /// @param motmagn_provider Provider for motion magnitude
    /// @param motang_provider Provider for motion angle
    virtual void create_motion_providers(ip_ImageProvider * image_provider,
            ip_ImageProvider *& motx_provider,
            ip_ImageProvider *& moty_provider,
            ip_ImageProvider *& motmagn_provider,
            ip_ImageProvider *& motang_provider);

    /// Deletes motion providers
    /// @param motx_provider Provider for X motion component
    /// @param moty_provider Provider for Y motion component
    /// @param motmagn_provider Provider for motion magnitude
    /// @param motang_provider Provider for motion angle
    virtual void delete_motion_providers(ip_ImageProvider *& motx_provider,
            ip_ImageProvider *& moty_provider,
            ip_ImageProvider *& motmagn_provider,
            ip_ImageProvider *& motang_provider);

    /// Obtain the most recent image, recompute motion components
    /// @param image Image to write results to, not used for group providers
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    void dump_pyramid(CMotion2DPyramid& pyramid, unsigned char * image,
            const std::string& prefix, int counter);
    void dump_estimator(const ip_RoiWindow& roi, double origin_row,
            double origin_column, CMotion2DImage<unsigned char>& support,
            const std::string& prefix, int counter);

    ImageProcessing::ip_Dense2DMotionProcessorInriaConfig m_Config;

    CMotion2DModel     m_MotionParametricModel; // motion parametric model
    CMotion2DEstimator m_MotionEstimator;       // motion estimator
    CMotion2DPyramid   m_Pyramid1;              // first image pyramid
    CMotion2DPyramid   m_Pyramid2;              // second image pyramid
    CMotion2DWarping   m_Warping;               // warping
    IplImage *         m_Image_8U;              // CMotion2D format

    // if set to true, motion model performs illumination variation estimation
    bool m_PreviousPyramidBuilt;

}; // class ip_Dense2DMotionProcessorInria

} // namespace ImageProcessing

#endif

#endif // __IP_DENSE2DMOTIONPROCESSORINRIA_H__
