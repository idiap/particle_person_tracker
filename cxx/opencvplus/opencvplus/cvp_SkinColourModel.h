// Copyright (c) 2011-2020 Idiap Research Institute
//
// cvp_SkinColourModel - class to represent a skin colour model
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __CVP_SKINCOLOURMODEL_H__
#define __CVP_SKINCOLOURMODEL_H__

#include <iostream>

namespace OpenCvPlus{

/// @brief Class to represent a skin colour model
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

struct cvp_SkinColourModel {

    static const cvp_SkinColourModel DEFAULT;

    float mGreenMean;
    float mGreenVar;
    float mRedMean;
    float mRedVar;
    float mGreenRedCovar;

};

float distance(const cvp_SkinColourModel& model, float green, float red);

} // namespace ImageProcessing

/// Output operator.
///
/// @param os Output stream to put the head pose structure
/// @param cm Colour model structure to stream
/// @return A reference to the output stream.
std::ostream& operator<<(std::ostream& os,
        const OpenCvPlus::cvp_SkinColourModel& cm);

/// Elementwise multiplication operator.
///
/// @param cm1 Multiplicand colour model
/// @param cm2  Multiplier colour model
/// @return The elementwise product of the colour model structures.
OpenCvPlus::cvp_SkinColourModel
operator*(const OpenCvPlus::cvp_SkinColourModel& cm1,
        const OpenCvPlus::cvp_SkinColourModel& cm2);

/// Elementwise multiplication operator.
///
/// @param cm1 Multiplicand colour model
/// @param cm2 Multiplier colour model
/// @return The elementwise product of the colour model structure.
template<typename T>
OpenCvPlus::cvp_SkinColourModel
operator*(const OpenCvPlus::cvp_SkinColourModel& cm,
        const T& v) {
    OpenCvPlus::cvp_SkinColourModel result;
    result.mGreenMean     = cm.mGreenMean * v;
    result.mGreenVar      = cm.mGreenVar  * v;
    result.mRedMean       = cm.mRedMean   * v;
    result.mRedVar        = cm.mRedVar    * v;
    result.mGreenRedCovar = cm.mGreenRedCovar * v;
    return result;
} // *

/// Elementwise multiplication operator.
///
/// @param cm Multiplicand colour model
/// @param v Multiplier value
/// @return The elementwise product of the colour model structure.
template<typename T>
OpenCvPlus::cvp_SkinColourModel
operator*(const T& v,
        const OpenCvPlus::cvp_SkinColourModel& cm) {
    OpenCvPlus::cvp_SkinColourModel result;
    result.mGreenMean     = v * cm.mGreenMean;
    result.mGreenVar      = v * cm.mGreenVar;
    result.mRedMean       = v * cm.mRedMean;
    result.mRedVar        = v * cm.mRedVar;
    result.mGreenRedCovar = v * cm.mGreenRedCovar;
    return result;
} // *

/// Elementwise addition operator.
///
/// @param cm1 Augend colour model
/// @param cm2  Addend colour model
/// @return The elementwise sum of the colour model structures.
OpenCvPlus::cvp_SkinColourModel
operator+(const OpenCvPlus::cvp_SkinColourModel& cm1,
        const OpenCvPlus::cvp_SkinColourModel& cm2);

/// Elementwise addition operator.
///
/// @param cm1 Augend colour model
/// @param v  Addend value
/// @return The elementwise sum of the colour model structure.
template<typename T>
OpenCvPlus::cvp_SkinColourModel
operator+(const OpenCvPlus::cvp_SkinColourModel& cm,
        const T& v) {
    OpenCvPlus::cvp_SkinColourModel result;
    result.mGreenMean     = cm.mGreenMean + v;
    result.mGreenVar      = cm.mGreenVar  + v;
    result.mRedMean       = cm.mRedMean   + v;
    result.mRedVar        = cm.mRedVar    + v;
    result.mGreenRedCovar = cm.mGreenRedCovar + v;
    return result;
} // +

/// Elementwise addition operator.
///
/// @param cm1 Augend colour model
/// @param v  Addend value
/// @return The elementwise sum of the colour model structure.
template<typename T>
OpenCvPlus::cvp_SkinColourModel
operator+(const T& v, const OpenCvPlus::cvp_SkinColourModel& cm) {
    OpenCvPlus::cvp_SkinColourModel result;
    result.mGreenMean     = v + cm.mGreenMean;
    result.mGreenVar      = v + cm.mGreenVar;
    result.mRedMean       = v + cm.mRedMean;
    result.mRedVar        = v + cm.mRedVar;
    result.mGreenRedCovar = v + cm.mGreenRedCovar;
    return result;
} // +

#endif // __CVP_SKINCOLOURMODEL_H__
