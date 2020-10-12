// Copyright (c) 2011-2020 Idiap Research Institute
//
// ModelDumperCs_TA2Format - class to dump main model data
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __MODELDUMPERCS_TA2FORMAT_H__
#define __MODELDUMPERCS_TA2FORMAT_H__

// LOCAL INCLUDES
#include "MainModel_ColourSegmentationTracker.h"                 // main model

/// @brief Class to dump main model data in TA2 annotation format
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    25.11.2011

class ModelDumperCs_TA2Format {

public:
    // LIFECYCLE

    /// Constructor
    ModelDumperCs_TA2Format(const std::string& filename,
            const MainModel_ColourSegmentationTracker* model);
    /// Destructor
    ~ModelDumperCs_TA2Format();

    // OPERATIONS
    void update();

private:

    void  dump_tracker(BICV::bicv_ColourSegmentationTracker * tracker,
            const boost::posix_time::time_duration& timestamp);

    const MainModel_ColourSegmentationTracker * m_Model;
    std::ofstream m_FileStream;
    // KLUDGE: should be removed after fixing timestamp problem
    int m_ImageCount;

};

#endif // __MODELDUMPERCS_TA2FORMAT_H__
