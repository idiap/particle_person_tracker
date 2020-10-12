// Copyright (c) 2011-2020 Idiap Research Institute
//
// ImageProviderRegistry - contains runtime instances of image providers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/foreach.hpp>               // foreach loop
#include <iostream>                        // IO

// LOCAL INCLUDES
#include <image_processing/ip_ImageProviderRegistry.h> // declaration of this
#include <image_processing/ip_ImageProviderGroup.h>    // image provider group
#include <image_processing/ip_Exceptions.h>            // module exceptions

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_ImageProviderRegistry::ip_ImageProviderRegistry() {
} // ImageProviderRegistry

void ip_ImageProviderRegistry::register_provider(ip_ImageProvider * provider) {
    cout << "register " << provider->id() << " : " << provider << endl;
    register_provider_recursive(provider);
    mProviderList.push_back(provider);
} // register_provider

void ip_ImageProviderRegistry::register_provider_recursive(
        ip_ImageProvider * provider) {
    // do not register any helper provider that is not identified by its ID
    if (provider->id() == IP_IMG_PROVIDER_HISTOGRAM) {
        return;
    }

    // if provider is in the list or in the registry throw an exception
    if (find(mProviderList.begin(), mProviderList.end(), provider) !=
            mProviderList.end()) {
        throw ip_Exception("Attempt to register an image provider that is "
                "already in the provider list!");
    } else if (mProviderRegistry.find(provider->id()) != mProviderRegistry.end()) {
        throw ip_Exception("Attempt to register an image provider that is "
                "already in the registry!");
    // if provider is a group, register all its components
    } else if (provider->id() == IP_IMG_PROVIDER_GROUP) {
        ip_ImageProviderGroup * provider_group =
                dynamic_cast<ip_ImageProviderGroup*>(provider);
        BOOST_FOREACH(ip_ImageProvider* pprovider, provider_group->providers()) {
            register_provider_recursive(pprovider);
        }
    // if provider is unknown, throw an exception
    } else if (provider->id() == IP_IMG_PROVIDER_UNKNOWN) {
        throw ip_Exception(
                "Attempt to register an image provider of unknown type!");
    // for regular provider put it into the registry
    } else {
        cout << "insert provider " << provider->id() << endl;
        mProviderRegistry.insert(pair<ip_ImageProviderType, ip_ImageProvider*>
            (provider->id(), provider));
    }
}

void ip_ImageProviderRegistry::unregister_provider(ip_ImageProvider * provider) {

    cout << "unregister " << provider->id() << " : " << provider << endl;

    list<ip_ImageProvider*>::iterator ii = find(mProviderList.begin(),
        mProviderList.end(), provider);
    if (ii != mProviderList.end()) {
        mProviderList.erase(ii);
    }

    // attempt to remove the provider from the registry
    if ((mProviderRegistry.find(provider->id()) != mProviderRegistry.end()) &&
        (mProviderRegistry[provider->id()] == provider)) {
        mProviderRegistry.erase(provider->id());
    // if provider is a group, unregister all its components
    } else if (provider->id() == IP_IMG_PROVIDER_GROUP) {
        ip_ImageProviderGroup * provider_group =
                dynamic_cast<ip_ImageProviderGroup*>(provider);
        BOOST_FOREACH(ip_ImageProvider* pprovider, provider_group->providers()) {
            unregister_provider(pprovider);
        }
    }

} // unregister_provider

ip_ImageProvider* ip_ImageProviderRegistry::provider(
        ip_ImageProviderType provider_type) {
    if (mProviderRegistry.find(provider_type) != mProviderRegistry.end()) {
        return mProviderRegistry[provider_type];
    } else {
        throw ip_Exception("Attempt to get a provider of unregistered type!");
    }

} // provider

std::list<ip_ImageProvider*>& ip_ImageProviderRegistry::providers() {
    return mProviderList;
} // providers

} // namespace ImageProcessing
