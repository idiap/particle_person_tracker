/**
 * @file src/vfoa_target.cc
 * @date 07 March 2013
 * @author Salim Kayal <salim.kayal@idiap.ch>
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Class to describe a target of visual focus of attention
 *
 * Copyright (C) 2011-2012 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include <assert.h>
#include <cmath>
#include <iostream>

#include "vfoa_target.h"
#include "person.h"

using namespace std;

/////////////////////////////// CONSTANTS ////////////////////////////////////

static const int UNFOCUSED_OBJECT_ID = -1;
//static const standard_deviation_t DEFAULT_STD(utils::PI/18,utils::PI/18,0);
//static const standard_deviation_t DEFAULT_UNFOCUSSED_STD(utils::PI/6,
//    utils::PI/6,0);
static const standard_deviation_t DEFAULT_TARGET_STD =
        standard_deviation_t(utils::PI / 180 * 12, utils::PI / 180 * 13, utils::PI/18);
static const float C_PAN = 1.0;
static const float C_TILT = 1.0;
static const float UNFOCUSED_PRIOR =
    (1.0 / (2 * utils::PI * DEFAULT_TARGET_STD.pan)) *
    (1.0 / (2 * utils::PI * DEFAULT_TARGET_STD.tilt)) *
    exp(-0.5 * (C_PAN * C_PAN + C_TILT * C_TILT));
static const float PRIOR_MIN = 0.001;
static const float ALPHA_PAN(0.6);
static const float ALPHA_TILT(0.4);
static const float ALPHA_H_PAN(0);
static const float ALPHA_H_TILT(0);

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

// create default parameters to be used with object/person targets
static VfoaTargetParameters create_default_target_parameters();

// create default parameters to be used for the unfocused target
static VfoaTargetParameters create_default_unfocused_parameters();

/////////////////////////////// PUBLIC ///////////////////////////////////////

const VfoaTargetParameters VfoaTarget::DEFAULT_VFOA_TARGET_PARAMETERS =
        create_default_target_parameters();

const VfoaTargetParameters VfoaTarget::DEFAULT_UNFOCUSED_TARGET_PARAMETERS =
        create_default_target_parameters();

VfoaTarget::VfoaTarget(const target_id_t & rId, const type_t & rType,
    const location_t &rLocation, const Person & rOwner,
    const VfoaTargetParameters & rParams /* = DEFAULT_VFOA_TARGET_PARAMETERS */)
    : cId(rType, rId), m_Owner(rOwner), m_Params(rParams),
      m_Location(rLocation) {

    bool is_ok = ComputeMean();
    if (is_ok) {
        ComputeGase();
        ComputePrior();
    } else {
        mPrior = PRIOR_MIN;
    }

} // VfoaTarget

VfoaTarget::VfoaTarget(const person_info_t &rPersonInfo,
    const Person & rOwner, const VfoaTargetParameters & rParams
    /* = DEFAULT_VFOA_TARGET_PARAMETERS */)
    : cId(PERSON_INFO, rPersonInfo.id), m_Owner(rOwner),
    m_Params(rParams), m_Location(rPersonInfo.loc) {
    bool is_ok = ComputeMean();
    if (is_ok) {
        ComputeGase();
        ComputePrior();
    } else {
        mPrior = PRIOR_MIN;
    }
} // VfoaTarget

VfoaTarget::VfoaTarget(const object_info_t &rObjectInfo, const Person & rOwner,
    const VfoaTargetParameters & rParams /* = DEFAULT_VFOA_TARGET_PARAMETERS */)
    : cId(OBJECT_INFO, rObjectInfo.id), m_Owner(rOwner),
    m_Params(rParams), m_Location(rObjectInfo.loc) {
    bool is_ok = ComputeMean();
    if (is_ok) {
        ComputeGase();
        ComputePrior();
    } else {
        mPrior = PRIOR_MIN;
    }
} // VfoaTarget

VfoaTarget::VfoaTarget(const Person & rOwner,
    const VfoaTargetParameters & rParams
    /* = DEFAULT_UNFOCUSED_TARGET_PARAMETERS */ ) :
    cId(UNFOCUSSED_VALUE, UNFOCUSED_OBJECT_ID), m_Owner(rOwner),
    m_Params(rParams) {
    mPrior = UNFOCUSED_PRIOR;
} // VfoaTarget

void VfoaTarget::UpdateTarget(const location_t &rLocation,
    const standard_deviation_t &rSigma){
  // bool is_ok = true;
  if (cId.type != UNFOCUSSED_VALUE){
    m_Params.m_StandardDeviation = rSigma;
    m_Location = rLocation;
    bool is_ok = ComputeMean();
    if(is_ok){
      ComputeGase();
      ComputePrior();
    } else {
      mPrior = PRIOR_MIN;
    }
  }
}

bool VfoaTarget::operator==(const VfoaTarget &rVfoaTarget){
  if(cId.id == rVfoaTarget.cId.id &&
      cId.type == rVfoaTarget.cId.type){
    return true;
  }
  return false;
}

/////////////////////////////// PRIVATE //////////////////////////////////////

bool VfoaTarget::ComputeMean() {
  //here we use the hypothesis that the camera is in 0,0,0 and that the ox,oy
  //plane is horizontal
  //assert(mRefLocation->x != mLocation.x || mRefLocation->y != mLocation.x ||
  //    mRefLocation->z != mLocation.z);
    location_t owner_loc = m_Owner.m_CurrentPersonInfo.loc;
    if ((owner_loc.x != m_Location.x) || (owner_loc.y != m_Location.y) ||
        (owner_loc.z != m_Location.z)) {
        location_t camera_loc;
        basis_t look_camera=utils::make_basis_from_positions(camera_loc,
                owner_loc);
        basis_t look_target=utils::make_basis_from_positions(m_Location,
                owner_loc);
        basis_t camera_to_target = utils::calculate_rotation(look_camera,
                look_target);
        mMean = utils::base_to_euler_YZX(camera_to_target);
        if (mMean.pan == mMean.pan && mMean.tilt == mMean.tilt &&
                mMean.roll == mMean.roll){
            return true;
        } else {
            return false;
        }
    }
//    std::cerr<<"Warning : target id "<<cId<<" is positionned at reference person."<<
//            "lowering probability to near zero"<<std::endl;
    return false;
}

void VfoaTarget::ComputeGase() {
  // assert mMean is not nan
  assert(mMean.pan == mMean.pan && mMean.tilt == mMean.tilt &&
      mMean.roll == mMean.roll);

  pose_t u_h1;
  const pose_t& ref_factor = m_Params.m_ReferenceFactor;
  const pose_t& ref_head_pose = m_Owner.m_ReferenceHeadPose;
  u_h1.pan = ref_factor.pan * mMean.pan +
          (1 - ref_factor.pan) * ref_head_pose.pan;
  u_h1.tilt = ref_factor.tilt * mMean.tilt +
          (1 - ref_factor.tilt) * ref_head_pose.tilt;

  const pose_t& prev_factor = m_Params.m_PreviousHeadPoseFactor;
  const pose_t& prev_head_pose = m_Owner.m_PreviousHeadPose;
  if (mMean.pan > 0) {
    if (prev_head_pose.pan < mMean.pan) {
      mGase.pan = u_h1.pan;
    } else {
      float pan_prov = u_h1.pan +
              prev_factor.pan * (prev_head_pose.pan - u_h1.pan);
      mGase.pan = (mMean.pan < pan_prov) ? mMean.pan : pan_prov;
    }
  } else {
    if (prev_head_pose.pan > mMean.pan) {
      float pan_prov =u_h1.pan +
              prev_factor.pan * (prev_head_pose.pan - u_h1.pan);
      mGase.pan = (mMean.pan > pan_prov) ? mMean.pan : pan_prov;
    } else {
      mGase.pan = u_h1.pan;
    }
  }

  if (mMean.tilt > 0){
    if (prev_head_pose.tilt < mMean.tilt) {
      mGase.tilt = u_h1.tilt;
    } else {
      float tilt_prov =u_h1.tilt +
              prev_factor.tilt * (prev_head_pose.tilt - u_h1.tilt);
      mGase.tilt = (mMean.tilt < tilt_prov) ? mMean.tilt : tilt_prov;
    }
  } else {
    if (prev_head_pose.tilt > mMean.tilt) {
      float tilt_prov = u_h1.tilt +
              prev_factor.tilt * (prev_head_pose.tilt - u_h1.tilt);
      mGase.tilt = (mMean.tilt > tilt_prov) ? mMean.tilt : tilt_prov;
    } else {
      mGase.tilt = u_h1.tilt;
    }
  }
}

void VfoaTarget::ComputePrior(){
    const standard_deviation_t& sigma = m_Params.m_StandardDeviation;
    const pose_t& head_pose = m_Owner.m_CurrentPersonInfo.head_pose;

    if (isnan(mGase.pan) || isnan(sigma.pan) || isnan(mGase.tilt) ||
        isnan(sigma.tilt)) {
        mPrior = PRIOR_MIN;
    } else {
        boost::math::normal_distribution<float> normal_pan_dist(mGase.pan,
                sigma.pan);
        boost::math::normal_distribution<float> normal_tilt_dist(mGase.tilt,
                sigma.tilt);
        mPrior = max(boost::math::pdf(normal_pan_dist, head_pose.pan)*
                     boost::math::pdf(normal_tilt_dist, head_pose.tilt),
                     PRIOR_MIN);
    }
}

//void VfoaTarget::PrintToFile(){
//  *cOutputFile<<"target identifier : ( type = "<<cId.type<<", id = "<<cId.id;
//  *cOutputFile<<" )"<<"\tlocation : "<<mLocation;
//  if (cId.type != UNFOCUSSED_VALUE){
//    *cOutputFile<<"\treference location : "<<*mRefLocation;
//    *cOutputFile<<"\treference pose : "<<*mReferencePosition;
//    *cOutputFile<< "\tprevious head pose : "<<*mPreviousHeadPose;
//  }
//  *cOutputFile<<"\tmean : "<<mMean<<"\tgase : "<<mGase;
//  *cOutputFile<<std::endl;
//}

/***********************public*************************/

/***********************getters*************************/
pose_t VfoaTarget::GetMean() const{ return mMean; }

pose_t VfoaTarget::GetGase() const{ return mGase; }

float VfoaTarget::GetPrior() const{ return mPrior; }

ostream& operator<<(ostream& out, const VfoaTargetParameters &rParams) {
    out << "SD: " << rParams.m_StandardDeviation
        << ", AC: " << rParams.m_ReferenceFactor
        << ", AH: " << rParams.m_PreviousHeadPoseFactor;
    return out;
} // operator<<

ostream& operator<<(ostream& out, const VfoaTarget &rVfoaTarget) {
    out << "Type: " << rVfoaTarget.cId.type
        << ", ID " << rVfoaTarget.cId.id << endl;
    out << "Parameters: " << rVfoaTarget.m_Params << endl;
    out << "Location: " << rVfoaTarget.m_Location << endl;
    out << "Mean: " << rVfoaTarget.mMean << endl;
    out << "Gaze: " << rVfoaTarget.mGase << endl;
    out << "Prior: " << rVfoaTarget.mPrior << endl;
    return out;
} // operator<<

/////////////////////////// LOCAL DECLARATIONS ///////////////////////////////

/* static */ VfoaTargetParameters create_default_target_parameters() {
    VfoaTargetParameters default_target_parameters;
    default_target_parameters.m_StandardDeviation = DEFAULT_TARGET_STD;
    default_target_parameters.m_ReferenceFactor =
            pose_t(ALPHA_PAN, ALPHA_TILT, 0.0);
    default_target_parameters.m_PreviousHeadPoseFactor =
            pose_t(ALPHA_H_PAN, ALPHA_H_TILT, 0.0);
    return default_target_parameters;
} // create_default_target_parameters

/* static */ VfoaTargetParameters create_default_unfocused_parameters() {
    VfoaTargetParameters default_unfocused_parameters;
    default_unfocused_parameters.m_StandardDeviation =
            standard_deviation_t(utils::PI/6, utils::PI/6, 0);
    default_unfocused_parameters.m_ReferenceFactor =
            pose_t(ALPHA_PAN, ALPHA_TILT, 0.0);
    default_unfocused_parameters.m_PreviousHeadPoseFactor =
            pose_t(ALPHA_H_PAN, ALPHA_H_TILT, 0.0);
    return default_unfocused_parameters;
} // create_default_target_parameters
