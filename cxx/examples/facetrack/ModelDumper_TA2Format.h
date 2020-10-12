// Copyright (c) 2011-2020 Idiap Research Institute
//
// ModelDumper_TA2Format - class to dump main model data
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __MODELDUMPER_TA2FORMAT_H__
#define __MODELDUMPER_TA2FORMAT_H__

// LOCAL INCLUDES
#include "ModelDumper.h"                                      // base class

/// @brief Class to dump main model data in TA2 annotation format
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    25.11.2011

class ModelDumper_TA2Format : public ModelDumper {

public:
    // LIFECYCLE

    /// Constructor
    ModelDumper_TA2Format(const std::string& filename, const MainModel* model);
    /// Destructor
    virtual ~ModelDumper_TA2Format();

    // OPERATIONS
    virtual void update();

private:

    void  dump_tracker(BICV::bicv_HeadPoseTracker * tracker,
            const boost::posix_time::time_duration& timestamp);

    std::ofstream m_FileStream;
    int m_ImageCount;

};

#endif // __MODELDUMPER_TA2FORMAT_H__
