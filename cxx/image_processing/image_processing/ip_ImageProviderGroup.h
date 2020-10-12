// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ImageProviderGroup - class to represent a provider that regroups
//                         several image providers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_IMAGEPROVIDERGROUP_H__
#define __IP_IMAGEPROVIDERGROUP_H__

// SYSTEM INCLUDES
#include <vector>                         // STL vector

// LOCAL INCLUDES
#include "ip_ImageProvider.h"             // base class declaration
#include "ip_ImageProviderType.h"         // provider types

namespace ImageProcessing {

/// @brief Class to represent a provider that regroups several image providers
///
/// This class defines a group that holds several image providers
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_ImageProviderGroup : public ip_ImageProvider {

    public:

    // OPERATIONS
  virtual std::string name() { return "[ip_ImageProviderGroup]"; }
    /// Overrides base class method, returns IP_IMG_PROVIDER_GROUP
    /// @return Group provider's ID IP_IMG_PROVIDER_GROUP
    /// @see ip_ImageProviderType
    virtual ip_ImageProviderType id() const { return IP_IMG_PROVIDER_GROUP; }

    /// Implements abstract method from base class, does nothing
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi) {}

    /// Implements method from base class, invalidates all siblings
    virtual void invalidate();

    /// Return all the image providers in the group for read/write access
    /// @return Collection of image providers
    std::vector<ip_ImageProvider*>& providers() {
        return mProviders;
    }

    /// Return all the image providers in the group for read-only access
    /// @return Collection of image providers
    const std::vector<ip_ImageProvider*>& providers() const {
        return mProviders;
    }

    protected:

    // OPERATIONS

    /// Adds a provider into the list of providers
    /// @return Collection of image providers
    void add_provider(ip_ImageProvider* provider);

    /// Removes a provider from the list of providers
    /// @return Collection of image providers
    void remove_provider(ip_ImageProvider* provider);

    private:

    std::vector<ip_ImageProvider*> mProviders;    // image providers

};

} // namespace ImageProcessing

#endif // __IP_IMAGEPROVIDERGROUP_H__
