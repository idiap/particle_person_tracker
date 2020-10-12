// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ImageProviderGroup - class to represent a provider that regroups
//                         several image providers
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <functional>                 // for mem_fun_ref
#include <algorithm>                  // for for_each
#include <iostream>                   // IO

// LOCAL INCLUDES
#include <image_processing/ip_ImageProviderGroup.h>    // declaration of this

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

/* virtual  */ void ip_ImageProviderGroup::invalidate() {
    ip_ImageProvider::invalidate();
    for_each(mProviders.begin(), mProviders.end(),
            mem_fun(&ip_ImageProvider::invalidate));
} // invalidate

/////////////////////////////// PROTECTED ////////////////////////////////////

void ip_ImageProviderGroup::add_provider(ip_ImageProvider* provider) {
       mProviders.push_back(provider);
} // add_provider

void ip_ImageProviderGroup::remove_provider(ip_ImageProvider* provider) {
    remove(mProviders.begin(), mProviders.end(), provider);
} // remove_provider

} // namespace ImageProcessing
