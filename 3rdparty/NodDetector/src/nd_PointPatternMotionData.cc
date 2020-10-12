/**
 * @file src/nd_PointPatternMotionData.cc
 * @date 04 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Representation of point motion data for a point pattern
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                         // boost foreach loop

// LOCAL INCLUDES
#include <noddetector/nd_PointPatternMotionData.h>   // declaration of this

using namespace std;

namespace NodDetector {

ostream& operator<<(ostream& out, const nd_PointPatternMotionData& data) {
    unsigned count = 1;
    BOOST_FOREACH(const nd_PointMotionData& mot, data.m_PointMotionDatas) {
        out << "Point " << count++ << " Motion Data [ " << endl;
        out << mot;
        out << ']' << endl;
    }
    return out;
} // operator<<

} // namespace NodDetector
