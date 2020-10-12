// Copyright (c) 2011-2020 Idiap Research Institute
//
// cvp_SkinColourModel - class to represent a skin colour model
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include "opencvplus/cvp_SkinColourModel.h"             // declaration of this

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static OpenCvPlus::cvp_SkinColourModel create_default_skin_colour_model();

///////////////////////////////// PUBLIC /////////////////////////////////////

namespace OpenCvPlus {

const cvp_SkinColourModel cvp_SkinColourModel::DEFAULT =
        create_default_skin_colour_model();

float distance(const cvp_SkinColourModel& model, float green, float red) {
    const float determinant = model.mGreenVar * model.mRedVar -
            model.mGreenRedCovar * model.mGreenRedCovar;
    const float sigmaInv11 = model.mRedVar / determinant;
    const float sigmaInv12 = -model.mGreenRedCovar / determinant;
    const float sigmaInv22 = model.mGreenVar / determinant;

    // evaluate the mahalanobis distance between
    // the model skin colour and the pixel colour
    green -= model.mGreenMean;
    red -= model.mRedMean;
    return (sigmaInv11 * green + sigmaInv12 * red) * green +
            (sigmaInv12 * green + sigmaInv22 * red) * red;

}

} // namespace ImageProcessing

std::ostream& operator<<(std::ostream& os,
        const OpenCvPlus::cvp_SkinColourModel& cm) {
    os << cm.mGreenMean << " " << cm.mGreenVar << " "
       << cm.mRedMean   << " " << cm.mRedVar   << " "
       << cm.mGreenRedCovar;
    return os;
} // <<

OpenCvPlus::cvp_SkinColourModel
operator*(const OpenCvPlus::cvp_SkinColourModel& cm1,
        const OpenCvPlus::cvp_SkinColourModel& cm2) {
    OpenCvPlus::cvp_SkinColourModel result;
    result.mGreenMean     = cm1.mGreenMean * cm2.mGreenMean;
    result.mGreenVar      = cm1.mGreenVar  * cm2.mGreenVar;
    result.mRedMean       = cm1.mRedMean   * cm2.mRedMean;
    result.mRedVar        = cm1.mRedVar    * cm2.mRedVar;
    result.mGreenRedCovar = cm1.mGreenRedCovar * cm2.mGreenRedCovar;
    return result;
} // *

OpenCvPlus::cvp_SkinColourModel
operator+(const OpenCvPlus::cvp_SkinColourModel& cm1,
        const OpenCvPlus::cvp_SkinColourModel& cm2) {
    OpenCvPlus::cvp_SkinColourModel result;
    result.mGreenMean     = cm1.mGreenMean + cm2.mGreenMean;
    result.mGreenVar      = cm1.mGreenVar  + cm2.mGreenVar;
    result.mRedMean       = cm1.mRedMean   + cm2.mRedMean;
    result.mRedVar        = cm1.mRedVar    + cm2.mRedVar;
    result.mGreenRedCovar = cm1.mGreenRedCovar + cm2.mGreenRedCovar;
    return result;
} // +

//////////////////////////// LOCAL DEFINITIONS ///////////////////////////////

OpenCvPlus::cvp_SkinColourModel create_default_skin_colour_model() {
    OpenCvPlus::cvp_SkinColourModel colour_model;

//    0.336175 0.379616 0.000086 0.000528 -0.000009
    colour_model.mGreenMean = 0.306742;
    colour_model.mGreenVar = 0.000239;
    colour_model.mRedMean = 0.389661;
    colour_model.mRedVar = 0.000976;
    colour_model.mGreenRedCovar = -0.000302;
//    colour_model.mGreenMean = 0.336175;
//    colour_model.mGreenVar = 0.000086;
//    colour_model.mRedMean = 0.379616;
//    colour_model.mRedVar = 0.000528;
//    colour_model.mGreenRedCovar = -0.000009;
    return colour_model;
} // create_default_skin_colour_model
