// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Dense2DMotionProcessor - abstract class to extract dense 2D motion vector
//                             fields from two consecutive images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_DENSE2DMOTIONPROCESSOR_H__
#define __IP_DENSE2DMOTIONPROCESSOR_H__

// LOCAL INCLUDES
#include "ip_ImageProviderGroup.h"  // base class

namespace ImageProcessing {

/// @brief Abstract class to extract dense 2D motion vector fields from
///        two consecutive images
///
/// This class introduces a general interface to extract dense 2D motion vector
/// fields from two consecutive images based on a sequential image provider.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_Dense2DMotionProcessor : public ip_ImageProviderGroup  {

    public:

    // LIFECYCLE

    /// Constructor to initialize dense 2D motion processor
    /// @param img_provider Sequential image provider
    ip_Dense2DMotionProcessor(ip_ImageProvider * img_provider);

    /// Destructor, deallocates partial image providers
    virtual ~ip_Dense2DMotionProcessor();

    // OPERATIONS

    /// Return input image provider
    /// @return Input image provider
    ip_ImageProvider* getImageProvider() {
        return m_ImageProvider;
    }

    /// Provider for the X motion component of the source image
    /// @return Provider for the X motion component of the source image
    ip_ImageProvider* getMotionXProvider();

    /// Provider for the Y motion component of the source image
    /// @return Provider for the Y motion component of the source image
    ip_ImageProvider* getMotionYProvider();

    /// Provider for motion magnitude values of the source image
    /// @return Provider for motion magnitude values of the source image
    ip_ImageProvider* getMotionMagnitudeProvider();

    /// Provider for motion angle values of the source image
    /// @return Provider for motion angle values of the source image
    ip_ImageProvider* getMotionAngleProvider();

    /// Reestimates the motion model for a given ROI
    /// @param roi Region of interest for which to estimate the motion
    virtual void reestimate4roi(const ip_RoiWindow& roi);

    /// Returns ROI for which the parameters are currently estimated
    /// @return ROI for which the parameters are currently estimated
    ip_RoiWindow getCurrentRoi() const;

    protected:

    /// Creates motion providers using source image information
    /// @param motx_provider Provider for X motion component
    /// @param moty_provider Provider for Y motion component
    /// @param motmagn_provider Provider for motion magnitude
    /// @param motang_provider Provider for motion angle
    virtual void create_motion_providers(ip_ImageProvider * image_provider,
            ip_ImageProvider *& motx_provider,
            ip_ImageProvider *& moty_provider,
            ip_ImageProvider *& motmagn_provider,
            ip_ImageProvider *& motang_provider) = 0;

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

    // verifies that all the providers are created and registered
    void verify_providers_initialized();
    // registers motion providers
    void register_motion_providers();
    // unregisters motion providers
    void unregister_motion_providers();

    // provider of source images
    ip_ImageProvider * m_ImageProvider;
    // provider of the resulting X motion component
    ip_ImageProvider * m_MotionXProvider;
    // provider of the resulting Y motion component
    ip_ImageProvider * m_MotionYProvider;
    // provider of the resulting motion magnitude
    ip_ImageProvider * m_MotionMagnitudeProvider;
    // provider of the resulting motion angle
    ip_ImageProvider * m_MotionAngleProvider;
    // flag indicating whether providers have been initialized
    bool m_ProvidersInitialized;
    // current ROI
    ip_RoiWindow m_CurrentRoi;

}; // class ip_Dense2DMotionProcessor

} // namespace ImageProcessing

#endif // __IP_DENSE2DMOTIONPROCESSOR_H__
