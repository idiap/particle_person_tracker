// Copyright (c) 2011-2020 Idiap Research Institute
//
// vfoa_CognitiveVfoaModel - model to compute visual focus of attention (VFOA)
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __VFOA_COGNITIVEVFOAMODEL_H__
#define __VFOA_COGNITIVEVFOAMODEL_H__

// LOCAL INCLUDES
#include "vfoa_General.h"                       // general module definitions

// SYSTEM INCLUDES
#include <string>                                // STL string
#include <list>                                  // STL list

// PROJECT INCLUDES
#include <bayes_image/bicv_HeadPoseTracker.h>         // head pose tracker
#include <bayes_filter/bf_DiscreteDistribution.h>     // discrete distribution

namespace VFOA {

    struct vfoa_CognitiveVfoaModelObjectInfo {
        // HP params, used if the object is not tracked and defined externally
        BICV::bicv_HeadPoseParameters mParameters;
        BICV::bicv_HeadPoseTracker * mTracker;
        std::string mName;
        unsigned mNumber;
        unsigned mId;
    };

    std::ostream& operator<<(std::ostream& out,
            const vfoa_CognitiveVfoaModelObjectInfo& obj);

    typedef BayesFilter::bf_DiscreteDistribution<
        vfoa_CognitiveVfoaModelObjectInfo> VfoaDistribution;

/// @brief A visual focus of attention (VFOA) computation model
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    11.01.2011

    class vfoa_CognitiveVfoaModel {

        public:

        /// Constructs a cognitive VFOA model based on a head pose domain
        /// @param head_pose_domain definition of the head pose domain
        vfoa_CognitiveVfoaModel(
            const OpenCvPlus::cvp_HeadPoseDiscreteDomain& head_pose_domain,
            int image_width, int image_height);

        void add_detected_object(BICV::bicv_HeadPoseTracker * tracker);
        void remove_detected_object(BICV::bicv_HeadPoseTracker * tracker);

        void add_predefined_objects(const std::string& filename);
        void remove_predefined_objects();

        const vfoa_CognitiveVfoaModelObjectInfo& object_info(
                BICV::bicv_HeadPoseTracker * tracker) const;

        const std::list<vfoa_CognitiveVfoaModelObjectInfo> objects() const {
            return mObjects;
        }

        int image_width() const {
            return mImageWidth;
        }

        int image_height() const {
            return mImageHeight;
        }

        VfoaDistribution compute_vfoa_distribution_Gaussian(
                BICV::bicv_HeadPoseTracker * tracker) const;

        VfoaDistribution compute_vfoa_distribution_Hellinger(
                BICV::bicv_HeadPoseTracker * tracker) const;

        private:

        VfoaDistribution compute_vfoa_distribution(
            BICV::bicv_HeadPoseTracker * tracker,
            float (vfoa_CognitiveVfoaModel::* pfun)(
                BICV::bicv_HeadPoseTracker * person,
                const VFOA::vfoa_CognitiveVfoaModelObjectInfo&
                    addressee,
                const OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose& ref_head_pose) const) const;

        float compute_probability_Gaussian(
            BICV::bicv_HeadPoseTracker * person,
            const VFOA::vfoa_CognitiveVfoaModelObjectInfo& addressee,
            const OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose& ref_head_pose) const;

        float compute_probability_Hellinger(
            BICV::bicv_HeadPoseTracker * person,
            const VFOA::vfoa_CognitiveVfoaModelObjectInfo& addressee,
            const OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose& ref_head_pose) const;

        int mImageWidth;
        int mImageHeight;
        unsigned mObjectsCount;
        const OpenCvPlus::cvp_HeadPoseDiscreteDomain& mHeadPoseDomain;
        std::list<vfoa_CognitiveVfoaModelObjectInfo> mObjects;

    };

} // namespace VFOA

#endif // __VFOA_COGNITIVEVFOAMODEL_H__
