// Copyright (c) 2010-2020 Idiap Research Institute
//
// cvp_HeadPoseDiscreteDomain - class to represent a set of discrete values
//                              of head pose and perform operations on the set
//                              in an optimal way (constant search)
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <list>                           // STL list
#include <algorithm>                      // STL algorithms

// LOCAL INCLUDES
#include "opencvplus/cvp_HeadPoseDiscreteDomain.h"  // declaration of this

using namespace std;
using namespace OpenCvPlus;

/////////////////////////////// CONSTANTS ////////////////////////////////////

static const cvp_HeadPoseDiscreteDomain::real DEFAULT_PAN_VALUES_ARRAY[] =
    {-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90};

static const cvp_HeadPoseDiscreteDomain::real DEFAULT_TILT_VALUES_ARRAY[] =
    {-60, -30, -15, 0, 15, 30, 60};

static const cvp_HeadPoseDiscreteDomain::real DEFAULT_ROLL_VALUES_ARRAY[] =
    {0};

/////////////////////////////// PUBLIC ///////////////////////////////////////

const cvp_HeadPoseDiscreteDomain::HeadPose
    cvp_HeadPoseDiscreteDomain::MAX_HEAD_POSE =
    cvp_HeadPoseDiscreteDomain::HeadPose(90, 60, 0);

const cvp_HeadPoseDiscreteDomain::HeadPose
    cvp_HeadPoseDiscreteDomain::MIN_HEAD_POSE =
    cvp_HeadPoseDiscreteDomain::HeadPose(-90, -60, 0);

const cvp_HeadPoseDiscreteDomain::HeadPose
    cvp_HeadPoseDiscreteDomain::DEFAULT_STEP_HEAD_POSE =
    cvp_HeadPoseDiscreteDomain::HeadPose(15, 15, 15);

const cvp_HeadPoseDiscreteDomain::ElementId
    cvp_HeadPoseDiscreteDomain::ID_INVALID = -1;

cvp_HeadPoseDiscreteDomain::cvp_HeadPoseDiscreteDomain() {

    // initialize pan/tilt/roll discrete value sets
    mPanValues.assign(DEFAULT_PAN_VALUES_ARRAY, DEFAULT_PAN_VALUES_ARRAY +
            sizeof(DEFAULT_PAN_VALUES_ARRAY) / sizeof(real));
    mTiltValues.assign(DEFAULT_TILT_VALUES_ARRAY, DEFAULT_TILT_VALUES_ARRAY +
            sizeof(DEFAULT_TILT_VALUES_ARRAY) / sizeof(real));
    mRollValues.assign(DEFAULT_ROLL_VALUES_ARRAY, DEFAULT_ROLL_VALUES_ARRAY +
            sizeof(DEFAULT_ROLL_VALUES_ARRAY) / sizeof(real));

    initialize_values_and_ids();

}// cvp_HeadPoseDiscreteDomain

cvp_HeadPoseDiscreteDomain::cvp_HeadPoseDiscreteDomain(
        const std::vector<real>& pan_values,
        const std::vector<real>& tilt_values,
        const std::vector<real>& roll_values) {

    // initialize pan/tilt/roll discrete value sets
    mPanValues.assign(pan_values.begin(), pan_values.end());
    mTiltValues.assign(tilt_values.begin(), tilt_values.end());
    mRollValues.assign(roll_values.begin(), roll_values.end());

    initialize_values_and_ids();

}// cvp_HeadPoseDiscreteDomain

cvp_HeadPoseDiscreteDomain::cvp_HeadPoseDiscreteDomain(
        const HeadPose& step_head_pose) {

    // initialize pan/tilt/roll discrete value sets
    mPanValues = generate_values(MIN_HEAD_POSE.pan(), step_head_pose.pan(),
            MAX_HEAD_POSE.pan());
    mTiltValues = generate_values(MIN_HEAD_POSE.tilt(), step_head_pose.tilt(),
            MAX_HEAD_POSE.tilt());
    mRollValues = generate_values(MIN_HEAD_POSE.roll(), step_head_pose.roll(),
            MAX_HEAD_POSE.roll());

    initialize_values_and_ids();

}// cvp_HeadPoseDiscreteDomain

cvp_HeadPoseDiscreteDomain::HeadPose cvp_HeadPoseDiscreteDomain::discretize(
        const cvp_HeadPoseDiscreteDomain::HeadPose& head_pose) const {

    cvp_HeadPoseDiscreteDomain::HeadPose result;
    result.pan(discretize(mPanValues, head_pose.pan()));
    result.tilt(discretize(mTiltValues, head_pose.tilt()));
    result.roll(discretize(mRollValues, head_pose.roll()));
    return result;

}// discretize

/////////////////////////////// PRIVATE //////////////////////////////////////

void cvp_HeadPoseDiscreteDomain::initialize_values_and_ids() {

    // sort discrete angle values and remove duplicates
    vector<real>::iterator ii;
    sort(mPanValues.begin(), mPanValues.end());
    ii = unique(mPanValues.begin(), mPanValues.end());
    mPanValues.resize(distance(mPanValues.begin(), ii));

    sort(mTiltValues.begin(), mTiltValues.end());
    ii = unique(mTiltValues.begin(), mTiltValues.end());
    mTiltValues.resize(distance(mTiltValues.begin(), ii));

    sort(mRollValues.begin(), mRollValues.end());
    ii = unique(mRollValues.begin(), mRollValues.end());
    mRollValues.resize(distance(mRollValues.begin(), ii));

    // initialize synchronized vectors of head pose values and their IDs
    mHeadPoseValues.resize(mPanValues.size() * mTiltValues.size() *
            mRollValues.size());
    mHeadPoseIds.resize(mHeadPoseValues.size());

    unsigned index = 0;
    for (vector<real>::const_iterator i_pan = mPanValues.begin();
            i_pan != mPanValues.end(); ++i_pan) {
        for (vector<real>::const_iterator i_tilt = mTiltValues.begin();
                i_tilt != mTiltValues.end(); ++i_tilt) {
            for (vector<real>::const_iterator i_roll = mRollValues.begin();
                    i_roll != mRollValues.end(); ++i_roll) {
                mHeadPoseValues[index] = HeadPose(*i_pan, *i_tilt,
                        *i_roll);
                mHeadPoseIds[index] = index;
                ++index;
            }
        }
    }

}// initialize_values_and_ids

std::vector<cvp_HeadPoseDiscreteDomain::real>
cvp_HeadPoseDiscreteDomain::generate_values(
        cvp_HeadPoseDiscreteDomain::real minval,
        cvp_HeadPoseDiscreteDomain::real step,
        cvp_HeadPoseDiscreteDomain::real maxval) const {

    list<real> values;
    for (real val = minval; val <= maxval; val += step) {
        values.push_back(val);
    }

    vector<real> result(values.size());
    copy(values.begin(), values.end(), result.begin());
    return result;

}// generate_values

unsigned cvp_HeadPoseDiscreteDomain::head_pose_index(
        const HeadPose& head_pose) const {

    unsigned idx_pan = distance(mPanValues.begin(),
        lower_bound(mPanValues.begin(), mPanValues.end(), head_pose.pan()));
    unsigned idx_tilt = distance(mTiltValues.begin(),
        lower_bound(mTiltValues.begin(), mTiltValues.end(), head_pose.tilt()));
    unsigned idx_roll = distance(mRollValues.begin(),
        lower_bound(mRollValues.begin(), mRollValues.end(), head_pose.roll()));

    if ((idx_pan < mPanValues.size()) && (idx_tilt < mTiltValues.size()) &&
            (idx_roll < mRollValues.size())) {
        return idx_pan * mTiltValues.size() * mRollValues.size() +
            idx_tilt * mRollValues.size() + idx_roll;
    } else {
        return 0;
    }

}// head_pose_index

unsigned cvp_HeadPoseDiscreteDomain::pan_index(const HeadPose& head_pose) const {
    return distance(mPanValues.begin(),
            lower_bound(mPanValues.begin(), mPanValues.end(), head_pose.pan()));
}

unsigned cvp_HeadPoseDiscreteDomain::tilt_index(const HeadPose& head_pose) const {
    return distance(mTiltValues.begin(),
        lower_bound(mTiltValues.begin(), mTiltValues.end(), head_pose.tilt()));
}

cvp_HeadPoseDiscreteDomain::real cvp_HeadPoseDiscreteDomain::discretize(
    const vector<cvp_HeadPoseDiscreteDomain::real>& array,
        real val) const {
    vector<real>::const_iterator ii = lower_bound(array.begin(), array.end(), val);
    if (ii == array.end()) {
        return array.back();
    } else if (ii == array.begin()) {
        return array.front();
    } else {
        real discr_val1 = *ii;
        real discr_val2 = *(ii - 1);
        return ((discr_val1 - val) < (val - discr_val2)) ? discr_val1 :
            discr_val2;
    }
}// discretize

/////////////////////////////// GLOBAL ///////////////////////////////////////
