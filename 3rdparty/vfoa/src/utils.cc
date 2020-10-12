#include "utils.h"
#include "vfoa_target.h"

/**
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

namespace utils{
  basis_t make_basis_from_positions(location_t rTargetLoc,
      location_t rPersonLoc){
    basis_t ret_basis;
    // OX - unit vector pointing from the person to the target
    float norm_vect = sqrt(dot(rTargetLoc - rPersonLoc,
            rTargetLoc - rPersonLoc));
    if (norm_vect <= std::numeric_limits<float>::epsilon()) {
        ret_basis.axis[0] = vector_3d_t(1, 0, 0);
        ret_basis.axis[1] = vector_3d_t(0, 1, 0);
        ret_basis.axis[2] = vector_3d_t(0, 0, 1);
        return ret_basis;
    }
    ret_basis.axis[0] = (rTargetLoc - rPersonLoc) / norm_vect;
    // OZ - unit vector, the result of projection of vector OX on the
    //      horizontal plane and 90 degrees rotation
    ret_basis.axis[2] = vector_3d_t(ret_basis.axis[0].y,
            -ret_basis.axis[0].x, 0);
    ret_basis.axis[2] /= sqrt(dot(ret_basis.axis[2], ret_basis.axis[2]));
    // OY = OX x OZ
    ret_basis.axis[1] = cross(ret_basis.axis[0], ret_basis.axis[2]);
    return ret_basis;
  }

  float dot(const vector_3d_t& loc1, const vector_3d_t& loc2){
    return float(loc1.x*loc2.x+loc1.y*loc2.y+loc1.z*loc2.z);
  }

  vector_3d_t cross(const vector_3d_t& loc1, const vector_3d_t& loc2){
    vector_3d_t ret;
    ret.x = loc1.y * loc2.z - loc1.z * loc2.y;
    ret.y = -(loc1.x * loc2.z - loc1.z * loc2.x);
    ret.z = loc1.x * loc2.y - loc1.y * loc2.x;
    return ret;
  }

  basis_t calculate_rotation(const basis_t& rBasis1, const basis_t& rBasis2) {
    basis_t ret_basis;
    for (int i = 0;i<3;i++){
      ret_basis.axis[i].x = utils::dot(rBasis1.axis[0],rBasis2.axis[i]);
      ret_basis.axis[i].y = utils::dot(rBasis1.axis[1],rBasis2.axis[i]);
      ret_basis.axis[i].z = utils::dot(rBasis1.axis[2],rBasis2.axis[i]);
    }
    return ret_basis;
  }

  pose_t base_to_euler_YZX(basis_t rBasis){
    pose_t ret_pose;
    if (rBasis.axis[0].y > 0.998){
      ret_pose.tilt = PI/2;
      ret_pose.roll = 0;
      ret_pose.pan = atan2(-rBasis.axis[2].x, rBasis.axis[1].x);
    }
    else if (rBasis.axis[0].y < -0.998){
      ret_pose.tilt = -PI/2;
      ret_pose.roll = 0;
      ret_pose.pan = atan2(rBasis.axis[2].x, rBasis.axis[1].x);
    }
    else{
      ret_pose.pan = atan2(rBasis.axis[0].z,rBasis.axis[0].x);
      ret_pose.tilt = asin(-rBasis.axis[0].y);
      ret_pose.roll = atan2(rBasis.axis[2].y,rBasis.axis[1].y);
    }
    return ret_pose;
  }

  distribution_t compute_transition_probabilities(
      const vfoa_target_list_t &rTargets, const distribution_t &rDistrib,
      const float* rTransitionMatrix){
    distribution_t ret;
    vfoa_target_list_t::const_iterator vfoa_target_it = rTargets.begin();
    const float *matrix_begin_address = rTransitionMatrix;
    for (size_t i = 0; i<rTargets.size(); i++){
      prob_element_t new_element(0,(*vfoa_target_it).cId);
      vfoa_target_it++;
      distribution_t::const_iterator prob_it = rDistrib.begin();
      for (size_t j = 0; j < rDistrib.size(); j++) {
        new_element.prob += *rTransitionMatrix*(*prob_it).prob;
        rTransitionMatrix++;
        prob_it++;
      }
      ret.push_front(new_element);
    }
    rTransitionMatrix = matrix_begin_address;
    return ret;
  }

  distribution_t compute_posterior(const vfoa_target_list_t &rTargets,
      const distribution_t &rTransitionProbabilities){
    distribution_t::const_iterator transition_prob_it = rTransitionProbabilities.begin();
    vfoa_target_list_t::const_iterator target_it;
    distribution_t new_probs;
    float normalizer = 0;
    for (target_it = rTargets.begin(); target_it != rTargets.end(); target_it++){
      prob_element_t new_element(0,(*target_it).cId);
      new_element.prob = (*target_it).GetPrior()*(*transition_prob_it).prob;
      normalizer += new_element.prob;
      new_probs.push_front(new_element);
    }
    distribution_t::iterator prob_it;
    for(prob_it = new_probs.begin(); prob_it != new_probs.end(); prob_it++){
      (*prob_it).prob /= normalizer;
    }
    return new_probs;
  }
  /*********************type checker****************************/
  template <> type_t Type<object_info_t>(){return OBJECT_INFO;}
  template <> type_t Type<person_info_t>(){return PERSON_INFO;}
}

ostream& operator<<(ostream& out, const vfoa_target_list_t & rVTlist) {
    out << "[target list :" << endl;
    std::ostream_iterator<VfoaTarget> out_it(out, ", ");
    std::copy(rVTlist.begin(), rVTlist.end(), out_it);
    out << "]";
    return out;
} // operator<<
