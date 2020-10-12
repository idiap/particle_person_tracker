// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Dense2DMotionProcessor - abstract class to extract dense 2D motion vector
//                             fields from two consecutive images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <image_processing/ip_Dense2DMotionProcessor.h> // declaration of this
#include <image_processing/ip_PartialImageProvider.h>   // for colour components

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_Dense2DMotionProcessor::ip_Dense2DMotionProcessor(
        ip_ImageProvider * img_provider)
        : m_ImageProvider(img_provider),
          m_MotionXProvider(0), m_MotionYProvider(0),
          m_MotionMagnitudeProvider(0), m_MotionAngleProvider(0),
          m_ProvidersInitialized(false) {

} // ip_Dense2DMotionProcessor

/* virtual */ ip_Dense2DMotionProcessor::~ip_Dense2DMotionProcessor() {

    if (m_ProvidersInitialized) {
        unregister_motion_providers();
        delete_motion_providers(m_MotionXProvider, m_MotionYProvider,
            m_MotionMagnitudeProvider, m_MotionAngleProvider);
        m_ProvidersInitialized = false;
    }

} // ~ip_Dense2DMotionProcessor

ip_ImageProvider* ip_Dense2DMotionProcessor::getMotionXProvider() {
    verify_providers_initialized();
    return m_MotionXProvider;
} // getMotionXProvider

ip_ImageProvider* ip_Dense2DMotionProcessor::getMotionYProvider() {
    verify_providers_initialized();
    return m_MotionYProvider;
} // getMotionYProvider

ip_ImageProvider* ip_Dense2DMotionProcessor::getMotionMagnitudeProvider() {
    verify_providers_initialized();
    return m_MotionMagnitudeProvider;
} // getMotionMagnitudeProvider

ip_ImageProvider* ip_Dense2DMotionProcessor::getMotionAngleProvider() {
    verify_providers_initialized();
    return m_MotionAngleProvider;
} // getMotionAngleProvider

/* virtual */ void
ip_Dense2DMotionProcessor::reestimate4roi(const ip_RoiWindow& roi) {
    m_CurrentRoi = roi;
    verify_providers_initialized();
} // reestimate4roi

/* virtual */ void
ip_Dense2DMotionProcessor::recompute_image(IplImage* image,
        const ip_RoiWindow& roi, boost::posix_time::ptime& time) {
    verify_providers_initialized();
} // recompute_image

/* static */ ip_RoiWindow ip_Dense2DMotionProcessor::getCurrentRoi() const {
    return m_CurrentRoi;
} // getCurrentRoi

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void ip_Dense2DMotionProcessor::delete_motion_providers(
    ip_ImageProvider *& motx_provider, ip_ImageProvider *& moty_provider,
    ip_ImageProvider *& motmagn_provider, ip_ImageProvider *& motang_provider) {

    delete motx_provider; motx_provider = 0;
    delete moty_provider; moty_provider = 0;
    delete motmagn_provider; motmagn_provider = 0;
    delete motang_provider; motang_provider = 0;

} // delete_motion_providers

//////////////////////////////// PRIVATE /////////////////////////////////////

void ip_Dense2DMotionProcessor::verify_providers_initialized() {
    if (!m_ProvidersInitialized) {
        create_motion_providers(m_ImageProvider,
                m_MotionXProvider, m_MotionYProvider,
                m_MotionMagnitudeProvider, m_MotionAngleProvider);
        register_motion_providers();
        m_ProvidersInitialized = true;
    }
} // verify_providers_initialized

void ip_Dense2DMotionProcessor::register_motion_providers() {
    add_provider(m_MotionXProvider);
    add_provider(m_MotionYProvider);
    add_provider(m_MotionMagnitudeProvider);
    add_provider(m_MotionAngleProvider);
} // register_motion_providers

void ip_Dense2DMotionProcessor::unregister_motion_providers() {
    remove_provider(m_MotionXProvider);
    remove_provider(m_MotionYProvider);
    remove_provider(m_MotionMagnitudeProvider);
    remove_provider(m_MotionAngleProvider);
} // unregister_motion_providers

} // namespace ImageProcessing
