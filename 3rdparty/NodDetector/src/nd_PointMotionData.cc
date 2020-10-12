/**
 * @file src/nd_PointMotionData.cc
 * @date 04 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Representation of point motion data
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                         // boost foreach loop

// LOCAL INCLUDES
#include <noddetector/nd_PointMotionData.h>          // declaration of this

using namespace std;

namespace NodDetector {

ostream& operator<<(ostream& out, const nd_PointMotionData& data) {
    ostream_iterator<nd_PointMotionData::RealType> out_it(out, " ");
    out << "X Motion Data [ ";
    copy(data.m_XMotionData.begin(), data.m_XMotionData.end(), out_it);
    out << ']' << endl;
    out << "Y Motion Data [ ";
    copy(data.m_YMotionData.begin(), data.m_YMotionData.end(), out_it);
    out << ']' << endl;
    return out;
} // operator<<

} // namespace NodDetector
