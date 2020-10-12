// Copyright (c) 2011-2020 Idiap Research Institute
//
// ImageProviderRegistry - contains runtime instances of image providers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_IMAGEPROVIDERREGISTRY_H__
#define __IP_IMAGEPROVIDERREGISTRY_H__

// SYSTEM INCLUDES
#include <map>                           // STL map container
#include <list>                          // STL list container

// LOCAL INCLUDES
#include "ip_ImageProvider.h"            // provider
#include "ip_ImageProviderType.h"        // provider type

namespace ImageProcessing {

/// @brief Class to keep runtime instances of image providers
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    07.02.2011

class ip_ImageProviderRegistry {

    public:

    // LIFECYCLE

    /// Default constructor
    ip_ImageProviderRegistry();

    // OPERATIONS

    /// Register a single provider or a provider group
    /// @param provider Provider or provider group to register
    void register_provider(ip_ImageProvider * provider);

    /// Unregister a single provider or a provider group
    /// @param provider Provider or provider group to remove from registry
    void unregister_provider(ip_ImageProvider * provider);

    /// Get registered provider by its type
    /// @param provider_type Provider type
    ip_ImageProvider* provider(ip_ImageProviderType provider_type);

    /// Get the list of registered providers
    /// @return The list of registered providers
    std::list<ip_ImageProvider*>& providers();

    private:

    // register an arbitrary provider, do recursion on group providers
    void register_provider_recursive(ip_ImageProvider * provider);

    // registry that contains all "leaf" providers, no groups
    std::map<ip_ImageProviderType, ip_ImageProvider*> mProviderRegistry;

    // list that contains providers as they are added (with groups)
    std::list<ip_ImageProvider*> mProviderList;

};

} // namespace ImageProcessing

#endif // __IP_IMAGEPROVIDERREGISTRY_H__
