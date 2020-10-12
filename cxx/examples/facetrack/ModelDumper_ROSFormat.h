// Copyright (c) 2011-2020 Idiap Research Institute
//
// ModelDumper_ROSFormat - class to dump main model data
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __MODELDUMPER_ROSFORMAT_H__
#define __MODELDUMPER_ROSFORMAT_H__

// SYSTEM INCLUDES
#include <ros/ros.h>

// LOCAL INCLUDES
#include "ModelDumper.h"                                      // base class

/// @brief Class to dump main model data in ROS format
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    30.11.2011

class ModelDumper_ROSFormat : public ModelDumper {

public:
    // LIFECYCLE

    /// Constructor
    ModelDumper_ROSFormat(const std::string& scope, const MainModel* model);
    /// Destructor
    virtual ~ModelDumper_ROSFormat();

    // OPERATIONS
    virtual void update();

private:

    int m_DataIdx;
    ros::NodeHandle m_NodeHandle;
    ros::Publisher m_Publisher;

};

#endif // __MODELDUMPER_ROSFORMAT_H__
