// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_GeneralTrackerState - class for a general particle filter state for
//                            an object tracker
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.


// SYSTEM INCLUDES
#include <cmath>                                         // std math functions

// LOCAL INCLUDES
#include <bayes_image/bicv_GeneralTrackerState.h>        // declaration of this

namespace BICV {
float distance_scaled(const BICV::bicv_TrackerParameters& hp1,
        const BICV::bicv_TrackerParameters& hp2) {
    float dist_x = fabs(hp1.m_TranslationX - hp2.m_TranslationX) *
            (0.5f / hp1.m_Scale + 0.5f / hp2.m_Scale);
    float dist_y = fabs(hp1.m_TranslationY - hp2.m_TranslationY) *
            (0.5f / (hp1.m_Scale * hp1.m_Excentricity) +
             0.5f / (hp2.m_Scale * hp2.m_Excentricity));
    return sqrt(dist_x * dist_x + dist_y * dist_y);
}

}

std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_TrackerParameters& hp) {
    os << "TX: " << hp.m_TranslationX <<
          ", TY: " << hp.m_TranslationY <<
          ", Scale: " << hp.m_Scale <<
          ", Exc: " << hp.m_Excentricity;
    return os;
}

/// Addition operator.
/// @param hp1 Augend tracker parameters structure
/// @param hp2 Addend tracker parameters structure
/// @return The elementwise sum of the two tracker parameters structures.
BICV::bicv_TrackerParameters operator+(
    const BICV::bicv_TrackerParameters& hp1,
    const BICV::bicv_TrackerParameters& hp2) {

    BICV::bicv_TrackerParameters result = hp1;
    result.m_Excentricity += hp2.m_Excentricity;
    result.m_Scale += hp2.m_Scale;
    result.m_TranslationX += hp2.m_TranslationX;
    result.m_TranslationY += hp2.m_TranslationY;
    return result;
}

BICV::bicv_TrackerParameters operator-(
    const BICV::bicv_TrackerParameters& s1,
    const BICV::bicv_TrackerParameters& s2) {

    BICV::bicv_TrackerParameters result = s1;
    result.m_Excentricity -= s2.m_Excentricity;
    result.m_Scale -= s2.m_Scale;
    result.m_TranslationX -= s2.m_TranslationX;
    result.m_TranslationY -= s2.m_TranslationY;
    return result;

} // -


/// Elementwise multiplication operator.
///
/// @param hp1 Multiplicand tracker parameters structure
/// @param hp2 Multiplier tracker parameters structure
/// @return The elementwise product of the two tracker parameters structures.
BICV::bicv_TrackerParameters operator*(
    const BICV::bicv_TrackerParameters& hp1,
    const BICV::bicv_TrackerParameters& hp2) {
    BICV::bicv_TrackerParameters result = hp1;
    result.m_Excentricity *= hp2.m_Excentricity;
    result.m_Scale *= hp2.m_Scale;
    result.m_TranslationX *= hp2.m_TranslationX;
    result.m_TranslationY *= hp2.m_TranslationY;
    return result;
}

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand tracker parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the tracker parameters structure.
BICV::bicv_TrackerParameters operator*(const BICV::bicv_TrackerParameters& hp,
        float val) {
    BICV::bicv_TrackerParameters result = hp;
    result.m_Excentricity *= val;
    result.m_Scale *= val;
    result.m_TranslationX *= val;
    result.m_TranslationY *= val;
    return result;
}

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand tracker parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the tracker parameters structure.
BICV::bicv_TrackerParameters operator*(float val,
        const BICV::bicv_TrackerParameters& hp) {
    BICV::bicv_TrackerParameters result = hp;
    result.m_Excentricity *= val;
    result.m_Scale *= val;
    result.m_TranslationX *= val;
    result.m_TranslationY *= val;
    return result;
}

std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_TrackerState& hp) {
    os << "Cur(" << hp.m_ParamsCur << "), Prev(" << hp.m_ParamsPrev << ")" <<
          ", label(" << hp.m_Label << ")";
    return os;
}

/// Addition operator.
/// @param hp1 Augend tracker parameters structure
/// @param hp2 Addend tracker parameters structure
/// @return The elementwise sum of the two tracker parameters structures.
BICV::bicv_TrackerState operator+(
    const BICV::bicv_TrackerState& hp1,
    const BICV::bicv_TrackerState& hp2) {
    BICV::bicv_TrackerState result(hp1);
    result.m_ParamsCur += hp2.m_ParamsCur;
    result.m_ParamsPrev += hp2.m_ParamsPrev;
    return result;
}

BICV::bicv_TrackerState operator-(
    const BICV::bicv_TrackerState& s1,
    const BICV::bicv_TrackerState& s2) {

    BICV::bicv_TrackerState result(s1);
    result.m_ParamsCur -= s2.m_ParamsCur;
    result.m_ParamsPrev -= s2.m_ParamsPrev;
    return result;
} // -

/// Multiplication operator.
/// @param hp1 Augend tracker parameters structure
/// @param hp2 Addend tracker parameters structure
/// @return The elementwise sum of the two tracker parameters structures.
BICV::bicv_TrackerState operator*(
    const BICV::bicv_TrackerState& hp1,
    const BICV::bicv_TrackerState& hp2) {
    BICV::bicv_TrackerState result(hp1);
    result.m_ParamsCur *= hp2.m_ParamsCur;
    result.m_ParamsPrev *= hp2.m_ParamsPrev;
    return result;
}

/// Multiplication operator.
/// @param hp1 Augend tracker parameters structure
/// @param hp2 Addend tracker parameters structure
/// @return The elementwise sum of the two tracker parameters structures.
BICV::bicv_TrackerState operator*(
    const BICV::bicv_TrackerState& hp1,
    float hp2) {
    BICV::bicv_TrackerState result(hp1);
    result.m_ParamsCur *= hp2;
    result.m_ParamsPrev *= hp2;
    return result;
}

/// Multiplication operator.
/// @param hp1 Augend tracker parameters structure
/// @param hp2 Addend tracker parameters structure
/// @return The elementwise sum of the two tracker parameters structures.
BICV::bicv_TrackerState operator*(float hp2,
    const BICV::bicv_TrackerState& hp1) {
    BICV::bicv_TrackerState result(hp1);
    result.m_ParamsCur *= hp2;
    result.m_ParamsPrev *= hp2;
    return result;
}

/// Multiplication operator.
/// @param hp1 Augend tracker parameters structure
/// @param hp2 Addend tracker parameters structure
/// @return The elementwise sum of the two tracker parameters structures.
BICV::bicv_TrackerState operator/( const BICV::bicv_TrackerState& hp1,
        float hp2) {
    BICV::bicv_TrackerState result(hp1);
    result.m_ParamsCur /= hp2;
    result.m_ParamsPrev /= hp2;
    return result;
}
