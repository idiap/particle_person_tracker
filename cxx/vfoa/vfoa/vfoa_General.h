// Copyright (c) 2011-2020 Idiap Research Institute
//
// vfoa_General - general definitions used in the VFOA module
//
// Author: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __VFOA_GENERAL_H__
#define __VFOA_GENERAL_H__

// SYSTEM INCLUDES
#include <functional>       // for unary function

namespace VFOA {

    /// real numbers type
    typedef double real;

    // Source: SGI STL library, file "stl_function.h"
    // select1st and select2nd are extensions:
    // they are not part of the standard.
    template <class _Pair>
    struct _Select1st : public std::unary_function<
        _Pair, typename _Pair::first_type> {
        const typename _Pair::first_type& operator()(const _Pair& __x) const {
            return __x.first;
        }
    };

    template <class _Pair>
    struct _Select2nd : public std::unary_function<
        _Pair, typename _Pair::second_type> {
        const typename _Pair::second_type& operator()(const _Pair& __x) const {
            return __x.second;
        }
    };

    template <class _Pair> struct select1st : public _Select1st<_Pair> {};
    template <class _Pair> struct select2nd : public _Select2nd<_Pair> {};

}

#endif // __VFOA_GENERAL_H__
