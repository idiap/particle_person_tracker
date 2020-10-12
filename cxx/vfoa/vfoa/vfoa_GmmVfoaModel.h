// Copyright (c) 2011-2020 Idiap Research Institute
//
// vfoa_GmmVfoaModel - model for visual focus of attention (VFOA) computation
//                     based on trained GMM model
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//          Elisa Ricci    (elisa.ricci@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __VFOA_GMMVFOAMODEL_H__
#define __VFOA_GMMVFOAMODEL_H__

// SYSTEM INCLUDES
#include <string>          // standard strings library
#include <map>             // standard associative map container
#include <list>            // standard set container
#include <vector>          // standard vector container
#include <boost/serialization/access.hpp> // private access for serialization

// PROJECT INCLUDES
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>  // head pose declaration

// LOCAL INCLUDES
#include "vfoa_General.h"  // general definitions for the VFOA module

namespace VFOA {

/// @brief A GMM-based visual focus of attention (VFOA) computation model
///
/// This class defines a visual focus of attention (VFOA) computation model
/// that uses a trained Gaussian mixture model (GMM) with outliers to predict
/// attention targets or detect the unfocused gaze with no particular target.
/// The unfocused gaze is assumed to be uniformly distributed over the pan-tilt
/// domain, whereas for a focused target the likelihood is given by a
/// multivariate Gaussian distribution with independent pan and tilt components.
/// This VFOA model uses the maximum likelihood (ML) principle to determine
/// the focus target.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @author  Elisa Ricci (elisa.ricci@idiap.ch)
/// @version 2.0
/// @date    11.01.2011

    class vfoa_GmmVfoaModel {

        public:

        /// Object ID type
        typedef unsigned int ObjectId;

        /// Constructs GMM VFOA model based on a head pose domain
        /// @param head_pose_domain definition of the head pose domain
        vfoa_GmmVfoaModel(const OpenCvPlus::cvp_HeadPoseDiscreteDomain *const
                head_pose_domain) : mHeadPoseDiscreteDomain(head_pose_domain) {}

        /// Storage class for the object information, includes object's name
        /// and mean and standard deviation values of head pose angles
        /// (pan and tilt) when looking at the object. Roll is assumed to be
        /// irrelevant to gaze direction.
        ///
        /// @note Strange logic: mean gaze and tilt are valid only for static
        /// scenes and for one particular object!
        ///
        /// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
        /// @author  Elisa Ricci (elisa.ricci@idiap.ch)
        /// @version 2.0
        /// @date    11.01.2011

        struct ObjectInfo {

            public:

                /// Create an object info structure
                /// @param name object's presentable name
                /// @param pan  mean pan angle when looking at the object
                /// @param tilt mean tilt angle when looking at the object
                /// @param stdpan standard deviation of pan angle
                /// when looking at the object
                /// @param stdtilt standard deviation of tilt angle
                /// when looking at the object
                /// @return object info structure with the unique ID
               static ObjectInfo create(const std::string& name,
                    real pan, real tilt, real stdpan, real stdtilt);

                /// Object's unique ID
                /// @return Object's unique ID
                ObjectId id() const { return mId; }

                /// Object presentable name
                std::string mName;
                /// Mean pan angle of a head when looking at the object
                real mMeanPan;
                /// Mean tilt angle of a head when looking at the object
                real mMeanTilt;
                /// Standard deviation of the pan angle of a head
                /// when looking at the object
                real mStddevPan;
                /// Standard deviation of the tilt angle of a head
                /// when looking at the object
                real mStddevTilt;

                static const ObjectInfo UNFOCUSED;

            private:

                // do not allow ObjectInfo to be constructed from outside
                // construct an object using the ID and data to store
                ObjectInfo(ObjectId id, const std::string& name,
                    real pan, real tilt, real stdpan, real stdtilt) :
                    mName(name), mMeanPan(pan), mMeanTilt(tilt),
                    mStddevPan(stdpan), mStddevTilt(stdtilt), mId(id) {}

                // hide the serialization functionality
                template<class Archive>
                void serialize(Archive & ar, const unsigned int ver);

                // give access to private fields to serialize properly
                friend class boost::serialization::access;

                // generate valid unique object IDs
                static ObjectId generateObjectId();

                // object ID
                ObjectId mId;

        };

        /// Compute VFOA with GMM using the ML criterion
        /// @param head_pose head pose to compute VFOA on
        /// @return object that is currently under attention
        vfoa_GmmVfoaModel::ObjectInfo
        compute_vfoa_ML(
            const OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose& head_pose);

        /// Compute VFOA using Hellinger distance between distributions
        /// H^2(P,Q) = \frac{1}{2} \int \left(
        ///     \sqrt{\frac{dP}{d\lambda}} - \sqrt{\frac{dQ}{d\lambda}}
        ///     \right)^2 d\lambda
        /// @param head_pose_distribution distribution over the head pose domain
        /// @return object that is currently under attention
        vfoa_GmmVfoaModel::ObjectInfo
        compute_vfoa_DD_Hellinger(const std::vector<VFOA::real>&
                head_pose_distribution);

        /// Add object information to the model
        /// @param obj_info object info to insert
        void add_object(const ObjectInfo& obj_info);

        /// Remove object information from the model
        /// @param obj_info object info to remove
        void remove_object(const ObjectInfo& obj_info);

        /// Number of objects currently in the model
        /// @return number of objects currently in the model
        size_t objects_number() const;

        /// Get all object information from the model
        /// @return list of object information structures
        std::list<ObjectInfo> objects() const;

        private:

        // Computes VFOA distribution for an object using object information
        std::vector<real> compute_vfoa_distribution(const ObjectInfo& obj_info);

        // associative map to keep object information
        std::map<ObjectId, ObjectInfo> mObjectPool;

        // associative map to keep object information
        std::map<ObjectId, std::vector<real> > mObjectHeadPoseDistributionPool;

        // discrete domain of valid head poses
        const OpenCvPlus::cvp_HeadPoseDiscreteDomain *const mHeadPoseDiscreteDomain;

    };

    /// Serialize VFOA model
    /// @param model VFOA model to serialize
    /// @param filename name of the file to serialize the model to
    void
    save_vfoa_model(const vfoa_GmmVfoaModel& model,
            const std::string& filename);

    /// Deserialize VFOA model
    /// @param model VFOA model to write the file contents to
    /// @param filename name of the file to read the model from
    void
    restore_vfoa_model(vfoa_GmmVfoaModel& model, const std::string& filename);

} // namespace VFOA

#endif // __VFOA_GMMVFOAMODEL_H__
