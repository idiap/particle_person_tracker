/**
 * @file cxx/examples/facetrack/NodDetector.cc
 * @date 15 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Detects nods for a tracked person
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <list>                                     // STL list

// PROJECT INCLUDES
#ifdef __ND_NOD_DETECTOR_FOUND__
#include <noddetector/nd_Utils.h>                   // for motion interpolation
#endif

// LOCAL INCLUDES
#include "NoddingDetector.h"                        // declaration of this

using namespace std;
using namespace BICV;
using namespace ImageProcessing;

#ifdef __ND_NOD_DETECTOR_FOUND__
using namespace NodDetector;
#endif

//#define DEBUG_TRACE

/////////////////////////////// PUBLIC ///////////////////////////////////////

NoddingDetector::NoddingDetector(const string& nod_detector_config_file) {
#ifdef __ND_NOD_DETECTOR_FOUND__
    try{
        m_NodDetector = nd_NodDetector::load(nod_detector_config_file);
    } catch(...) {
        m_NodDetector = 0;
    }

    if (m_NodDetector) {
        m_NodDetectorCache = m_NodDetector->create_cache();
        const double motion_data_frequency_Hz =
                m_NodDetector->motion_data_frequency();
        const double motion_data_size = static_cast<double>(
                m_NodDetector->motion_data_size());
        m_NodDetectorTimeWindow_MicroSec =
                motion_data_size / motion_data_frequency_Hz * 1e6;
        m_NodDetectorTimeStep_MicroSec = 1.0 / motion_data_frequency_Hz * 1e6;
    }
#endif
} // NoddingDetector

NoddingDetector::~NoddingDetector() {
#ifdef __ND_NOD_DETECTOR_FOUND__
    if (m_NodDetector) {
        delete m_NodDetector; m_NodDetector = 0;
        delete m_NodDetectorCache; m_NodDetectorCache = 0;
    }
#endif
} // ~NoddingDetector

bool NoddingDetector::detect_nod(const bicv_HeadPoseTracker& tracker) const {
#ifdef __ND_NOD_DETECTOR_FOUND__
    if (m_NodDetector) {

        const list<bicv_HeadPoseTracker::TimedMotionEstim>& motion_history =
            tracker.motion_estimation_history();
        if (motion_history.size() < 2) {
            // not enough data to estimate nods
            return false;
        }
        list<bicv_HeadPoseTracker::TimedMotionEstim>::const_iterator it =
                motion_history.begin();
        const double first_ts_microsec = it->first;
        const double last_ts_microsec = motion_history.back().first;

        if (first_ts_microsec + m_NodDetectorTimeWindow_MicroSec >
            last_ts_microsec) {
            // not enough data to estimate nods
            return false;
        }

        nd_PointPatternMotionData motion_data =
                create_motion_data(motion_history);
#ifdef DEBUG_TRACE
        cout << motion_data << endl;
#endif
        bool result = m_NodDetector->detect_nod(
                motion_data, *m_NodDetectorCache);
        return result;
    } else {
#ifdef DEBUG_TRACE
        cout << "Nod detector is null" << endl;
#endif
        return false;
    }
#else
#ifdef DEBUG_TRACE
        cout << "Nod detector was not compiled" << endl;
#endif
    return false;
#endif
}

/////////////////////////////// PRIVATE //////////////////////////////////////

#ifdef __ND_NOD_DETECTOR_FOUND__
nd_PointPatternMotionData
NoddingDetector::create_motion_data(
    const list<BICV::bicv_HeadPoseTracker::TimedMotionEstim>&
        motion_history) const {

#ifdef DEBUG_TRACE
    ostream_iterator<double> out_it(cout, " ");
    ostream_iterator<long> out_long_it(cout, " ");
#endif

    list<BICV::bicv_HeadPoseTracker::TimedMotionEstim>::const_iterator
        mot_hist_it;

    // initialise time stamps
    mot_hist_it = motion_history.begin();
    const list<BICV::bicv_HeadPoseTracker::TimedMotionEstim>::const_iterator
        mot_hist_end = motion_history.end();
    // time stamp 1
    const double prev_ts = mot_hist_it->first;
    ++mot_hist_it;
    // time stamps 2..N
    const size_t ts_size = motion_history.size() - 1;
    vector<double> ts(ts_size);
    vector<double>::iterator ts_it = ts.begin();
    while (mot_hist_it != mot_hist_end) {
        *ts_it++ = mot_hist_it->first;
        ++mot_hist_it;
    }

#ifdef DEBUG_TRACE
    cout << endl;
    cout << "Timestamps: " << endl;
    copy(ts.begin(), ts.end(), out_long_it);
    cout << endl;
#endif

    // construct destination pattern motion data
    const nd_PointPattern& point_pattern = m_NodDetector->point_pattern();
    const size_t motion_data_size = m_NodDetector->motion_data_size();
    const size_t points_num = point_pattern.m_Points.size();
    nd_PointPatternMotionData ptpattern_motion_data(
            points_num, motion_data_size);

    // initialise new time stamps
    const double last_ts = motion_history.back().first;
    vector<double> ts_new(motion_data_size + 1);
    const size_t ts_new_size = ts_new.size();
    double * ts_new_ptr = &ts_new[0];
    for (unsigned idx = 0; idx < ts_new_size; ++idx) {
        *ts_new_ptr++ = last_ts - (ts_new_size - 1 - idx) *
                m_NodDetectorTimeStep_MicroSec;
    }

#ifdef DEBUG_TRACE
    cout << "New timestamps: " << endl;
    copy(ts_new.begin(), ts_new.end(), out_long_it);
    cout << endl;
#endif

    // initialise values
    vector<double> values_x(ts_size);
    vector<double> values_y(ts_size);
    vector<double> values_new(ts_new_size);
    unsigned values_idx;
    double x_pt_rel, y_pt_rel, tx, ty, divx;

    for (unsigned idx = 0; idx < points_num; ++idx) {
        x_pt_rel = point_pattern.m_Points[idx].m_X;
        y_pt_rel = point_pattern.m_Points[idx].m_Y;
        values_idx = 0;
        for (mot_hist_it = motion_history.begin(), ++mot_hist_it;
             mot_hist_it != mot_hist_end; ++mot_hist_it) {
            const ip_MotionParameters& motion_parameters = mot_hist_it->second;
            tx = (motion_parameters.m_Flags[
                      ip_MotionParameters::TRANSLATION_X_IDX]) ?
                  motion_parameters.m_Parameters[
                      ip_MotionParameters::TRANSLATION_X_IDX] :
                  0;
            ty = (motion_parameters.m_Flags[
                      ip_MotionParameters::TRANSLATION_Y_IDX]) ?
                  motion_parameters.m_Parameters[
                      ip_MotionParameters::TRANSLATION_Y_IDX] :
                  0;
            divx = (motion_parameters.m_Flags[
                        ip_MotionParameters::DIV_X_IDX]) ?
                    motion_parameters.m_Parameters[
                        ip_MotionParameters::DIV_X_IDX] :
                    0;
            values_x[values_idx] = tx + divx *
                motion_parameters.m_BoundingBox.m_iWidth * (0.5 - x_pt_rel);
            values_y[values_idx] = ty + divx *
                motion_parameters.m_BoundingBox.m_iHeight * (0.5 - y_pt_rel);
            values_idx++;
        }
#ifdef DEBUG_TRACE
        cout << "Values X: " << endl;
        copy(values_x.begin(), values_x.end(), out_it);
        cout << endl;
        cout << "Values Y: " << endl;
        copy(values_y.begin(), values_y.end(), out_it);
        cout << endl;
#endif
        interpolate_motion(prev_ts, ts, values_x, ts_new, values_new);
        copy(values_new.begin() + 1, values_new.end(),
             ptpattern_motion_data.m_PointMotionDatas[idx].m_XMotionData.begin());
        interpolate_motion(prev_ts, ts, values_y, ts_new, values_new);
        copy(values_new.begin() + 1, values_new.end(),
             ptpattern_motion_data.m_PointMotionDatas[idx].m_YMotionData.begin());
#ifdef DEBUG_TRACE
        cout << "New values X: " << endl;
        copy(ptpattern_motion_data.m_PointMotionDatas[idx].m_XMotionData.begin(),
             ptpattern_motion_data.m_PointMotionDatas[idx].m_XMotionData.end(),
             out_it);
        cout << endl;
        cout << "New values Y: " << endl;
        copy(ptpattern_motion_data.m_PointMotionDatas[idx].m_YMotionData.begin(),
             ptpattern_motion_data.m_PointMotionDatas[idx].m_YMotionData.end(),
             out_it);
        cout << endl;
        cout << endl;
#endif
    }

    return ptpattern_motion_data;
} // create_motion_data
#endif
