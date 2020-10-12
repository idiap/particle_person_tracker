#ifndef UTILS_H
#define UTILS_H

/**
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include <list>
#include <math.h>
#include "types.h"

using namespace std;

class VfoaTarget;

typedef list<VfoaTarget> vfoa_target_list_t;
std::ostream& operator<<(std::ostream& out, const vfoa_target_list_t & rVTlist);

namespace utils{
  static const float PI(atan(1)*4);

  basis_t make_basis_from_positions(location_t rTargetLoc,
      location_t rPersonLoc);

  float dot(const vector_3d_t& loc1, const vector_3d_t& loc2);

  vector_3d_t cross(const vector_3d_t& loc1, const vector_3d_t& loc2);

  basis_t calculate_rotation(const basis_t& rBasis1, const basis_t& rBasis2);

  pose_t base_to_euler_YZX(basis_t rBasis);

  distribution_t compute_transition_probabilities(const vfoa_target_list_t &rTargets,
      const distribution_t &rDistrib, const float* mTransitionMatrix);

  distribution_t compute_posterior(const vfoa_target_list_t &rTargets,
      const distribution_t &rTransitionProbabilities);

  /*********************type checker****************************/
  template <typename T> type_t Type();

}// end namespace utils

#endif
