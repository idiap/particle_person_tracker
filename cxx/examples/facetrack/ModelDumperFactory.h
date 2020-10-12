/**
 * @file cxx/examples/facetrack/ModelDumperFactory.h
 * @date 29 January 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Creates model dumper by output source name (file, RSB, etc)
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __MODELDUMPERFACTORY_H__
#define __MODELDUMPERFACTORY_H__

// SYSTEM INCLUDES
#include <string>                    // STL string
#include <boost/program_options.hpp> // boost program options

// LOCAL INCLUDES
#include "ModelDumper.h"

/**
 * Model dumper factory that instantiates model dumpers
 * based on string representation of output source (XML file, RSB scope, etc.)
 */

class ModelDumperFactory {

    public:

    // LIFECYCLE

    /**
     * Access to the singleton factory object
     * @return Instance of the factory
     */
    static ModelDumperFactory * instance();

    // OPERATIONS

    /**
     * Creates an image provider based on string representation of data source
     * @param source Image provider source description. Can be video file name,
     *        file list name, single image, webcam, etc.)
     * @param width Preferred image width, -1 for the default width
     * @param height Preferred image height, -1 for the default height
     * @param fps Preferred fps rate, -1 for the default fps rate
     * @param offline Offline mode flag, TRUE by default
     * @return Instance of an image provider
     */
    ModelDumper * create_model_dumper(const std::string& output_name,
            const MainModel* model);

    private:

    // LIFECYCLE

    /// Constructor - hidden for a singleton class
    ModelDumperFactory();

    /// Copy constructor - not implemented!
    ModelDumperFactory(const ModelDumperFactory& );

    // OPERATIONS

    /// Assignment operator - not implemented
    void operator=(const ModelDumperFactory&);

};

#endif // __MODELDUMPERFACTORY_H__
