/**
 * @file cxx/examples/facetrack/ModelDumper.cc
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

// LOCAL INCLUDES
#include "ModelDumper.h"                             // declaration of this

using namespace std;

ModelDumper::
ModelDumper(const std::string& destination,
            const MainModel *model):
    m_Model(model),
    m_Destination(destination)
{
} // ModelDumper

ModelDumper::
~ModelDumper()
{
  m_Model = 0;
} // ~ModelDumper
