/**
 * @file cxx/image_processing/image_processing/ip_ImageProviderFactory.h
 * @date 27 November 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Creates image provider by source name (video file/webcam/single image)
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_IMAGEPROVIDERFACTORY_H__
#define __IP_IMAGEPROVIDERFACTORY_H__

// SYSTEM INCLUDES
#include <string>                    // STL string
#include <boost/program_options.hpp> // boost program options

// LOCAL INCLUDES
#include "ip_ImageProvider.h"        // base class

namespace ImageProcessing {

/**
 * Image provider factory that instantiates image providers
 * based on string representation of a source (video file, sequence of images,
 * webcam or any other stream)
 */

class ip_ImageProviderFactory {

    public:

    // LIFECYCLE

    /**
     * Access to the singleton factory object
     * @return Instance of the factory
     */
    static ip_ImageProviderFactory * instance();

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
    ip_ImageProvider*
    create_image_provider(const std::string& source,
                          int width = -1,
                          int height = -1,
                          double scale = 1.0,
                          int fps = -1,
                          bool offline = true);

    private:

    // LIFECYCLE

    /// Constructor
    ip_ImageProviderFactory();

    /// Copy constructor - not implemented!
    ip_ImageProviderFactory(const ip_ImageProviderFactory& );

    /// Assignment operator - not implemented
    void operator=(const ip_ImageProviderFactory&);

    // OPERATIONS

    // Check whether file is an image file
    bool is_image_file(const std::string& filename);

};

} // namespace ImageProcessing

#endif // __IP_IMAGEPROVIDERFACTORY_H__
