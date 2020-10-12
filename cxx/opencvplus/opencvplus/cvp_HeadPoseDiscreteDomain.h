// Copyright (c) 2010-2020 Idiap Research Institute
//
// cvp_HeadPoseDiscreteDomain - class to represent a set of discrete values
//                              of head pose and perform operations on the set
//                              in an optimal way (constant search)
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __CVP_HEADPOSEDISCRETEDOMAIN_H__
#define __CVP_HEADPOSEDISCRETEDOMAIN_H__

// SYSTEM INCLUDES
#include <vector>          // STL vector

// LOCAL INCLUDES
#include "cvp_HeadPose.h"  // definition of head pose

namespace OpenCvPlus {

/// @brief Class to represent a set of discrete values of head pose
///
/// This class defines a discrete set of head pose values that can be used
/// for efficient searches and data storage
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    19.01.2011

    class cvp_HeadPoseDiscreteDomain {

        public:

        // TYPES

        typedef cvp_HeadPose<float> HeadPose;
        typedef HeadPose::value_type real;
        typedef int ElementId;

        // LIFECYCLE

        /// Default constructor, predefined non-uniform steps for angles
        cvp_HeadPoseDiscreteDomain();

        /// Constructor that uses provided angle values to define the domain
        /// @param pan_values Angle values for pan
        /// @param tilt_values Angle values for tilt
        /// @param roll_values Angle values for roll
        cvp_HeadPoseDiscreteDomain(const std::vector<real>& pan_values,
                const std::vector<real>& tilt_values,
                const std::vector<real>& roll_values);

        /// Constructor from a head pose containing discretization step values
        /// @param stephp Step values for pan/tilt/roll
        cvp_HeadPoseDiscreteDomain(const HeadPose& stephp);

        // OPERATIONS

        /// Compute size of the set of head pose values
        /// @return Size of the set of head pose values
        size_t size() const {
            return mHeadPoseValues.size();
        }

        /// Get a discrete set of all possible head pose values packed into
        /// a vector. The order is fixed and is the same as the one returned by
        /// the ids() method.
        /// @return Discrete head pose values packed into a vector
        /// @see ids
        const std::vector<HeadPose>& values() const {
            return mHeadPoseValues;
        }

        /// Get a set of unique IDs that correspond to discrete head pose values
        /// contained in the set. The order is fixed and is the same as the one
        /// returned by the values() method.
        /// @return Head pose ID values packed into a vector
        /// @see values
        std::vector<ElementId> ids() const {
            return mHeadPoseIds;
        }

        /// Get a head pose value by its id.
        /// @param id A valid ID of a head pose.
        /// @return Head pose value that corresponds to the provided ID.
        HeadPose value(const ElementId& id) const {
            return mHeadPoseValues[id];
        }

        /// Get an ID of a head pose. Returns ID_INVALID, if the provided
        /// head pose is not in the set.
        /// @param head_pose A head pose for which to get an ID.
        /// @return The corresponding ID or ID_INVALID, if not in the set.
        ElementId id(const HeadPose& head_pose) const {
            if (contains(head_pose)) {
                return static_cast<int>(head_pose_index(head_pose));
            }
            return ID_INVALID;
        }

        /// Gets the head pose from the set which is the closest
        /// to the provided one in terms of Euclidian distance.
        /// The returned value is guaranteed to correspond to a valid ID.
        /// @param head_pose A head pose for which to get a discretization.
        /// @return A value from the set that is the closest to the provided
        ///     head pose.
        HeadPose discretize(const HeadPose& head_pose) const;

        unsigned pan_index(const HeadPose& head_pose) const;
        unsigned tilt_index(const HeadPose& head_pose) const;

        /// Checks whether the head pose is contained in the set.
        /// @param head_pose A head pose for which to check set membership.
        /// @return True if the provided head pose is in the set.
        bool contains(const HeadPose& head_pose) const {
            HeadPose head_pose_from_set =
                    mHeadPoseValues[head_pose_index(head_pose)];
            if (head_pose_from_set == head_pose) {
                return true;
            }
            return false;
        }

        /// Enlists all pan angle values
        /// @return Pan angle values used in the set.
        std::vector<real> pan_values() const {
            return mPanValues;
        }

        /// Enlists all tilt angle values
        /// @return Tilt angle values used in the set.
        std::vector<real> tilt_values() const {
            return mTiltValues;
        }

        /// Enlists all roll angle values
        /// @return Roll angle values used in the set.
        std::vector<real> roll_values() const {
            return mRollValues;
        }

        // CONSTANTS

        /// head pose with all angles set to maximum
        static const HeadPose MAX_HEAD_POSE;

        /// head pose with all angles set to minimum
        static const HeadPose MIN_HEAD_POSE;

        /// head pose containing default step values for all angles
        static const HeadPose DEFAULT_STEP_HEAD_POSE;

        /// used to distinguish a head pose that is not in the set
        static const ElementId ID_INVALID;

        private:

        // initialize head pose values and IDs
        // using discrete values for pan/tilt/roll
        void initialize_values_and_ids();

        // generate discrete values using min, step and max values
        std::vector<real> generate_values(real minval, real step,
                real maxval) const;

        // evaluate head pose index
        unsigned head_pose_index(const HeadPose& head_pose) const;

        // discretize values for a sorted vactor of angle values
        real discretize(const std::vector<real>& array, real val) const;

        // all head pose values packed into a vector
        std::vector<HeadPose> mHeadPoseValues;

        // head pose unique IDs values packed into a vector
        std::vector<ElementId> mHeadPoseIds;

        // discretized values of the pan angle
        std::vector<real> mPanValues;

        // discretized values of the tilt angle
        std::vector<real> mTiltValues;

        // discretized values of the roll angle
        std::vector<real> mRollValues;

    };

}

#endif // __CVP_HEADPOSEDISCRETEDOMAIN_H__
