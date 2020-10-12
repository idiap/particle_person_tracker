/**
 * @file cxx/examples/facetrack/ModelDumper.h
 * @date 29 January 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Base interface to create dumpers of model data
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __MODELDUMPER_H__
#define __MODELDUMPER_H__

// LOCAL INCLUDES
#include "MainModel.h"                                      // main model

/// @brief Base class to dump main model data
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    29.01.2013

class ModelDumper {

public:
    // LIFECYCLE

    /// Constructor
    /// @param destination Dump destination
    /// @param destination Main model to dump data from
    ModelDumper(const std::string& destination, const MainModel *model);

    /// Destructor
    virtual ~ModelDumper() = 0;

    // OPERATIONS

    /// Slot indicating that model has been updated and the new state can be
    /// dumped
    virtual void update() = 0;

protected:

    const MainModel * model() const {
        return m_Model;
    }

    const std::string& destination() const {
        return m_Destination;
    }

private:

    // main model that contains all trackers
    const MainModel * m_Model;
    // dump source
    std::string m_Destination;

};

#endif // __MODELDUMPER_H__
