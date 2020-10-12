/**
 * @file cxx/examples/facetrack/TrackerManagerData.h
 * @date 19 February 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Tracker manager data
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __TRACKERMANAGERDATA_H__
#define __TRACKERMANAGERDATA_H__

struct TrackerManagerData {

    float m_LhoodMean;
    float m_LhoodAmplitude;
    float m_VarMean;
    float m_VarAmplitude;
    float m_TrackMemMean;
    float m_TrackMemAmplitude;
    float m_LhoodDropMean;
    float m_LhoodDropAmplitude;
    float m_VarDropMean;
    float m_VarDropAmplitude;
    float m_VarIncMean;
    float m_VarIncAmplitude;

}; // TrackerManagerData

#endif /* __TRACKERMANAGERDATA_H__ */
