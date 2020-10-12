/**
 * @file cxx/opencvplus/opencvplus/cvp_IplDepthTraits.h
 * @date 01 June 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Mapping from numeric types into OpenCV IPL_DEPTH constants
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_IPLDEPTHTRAITS_H__
#define __CVP_IPLDEPTHTRAITS_H__

// SYSTEM INCLUDES
#include <boost/static_assert.hpp>   // for type sizeof assertions
#include <cv.h>                      // OpenCV depth constants

namespace OpenCvPlus {

    template<typename T>
    class cvp_IplDepthTraits {
    };

// SPECIALIZATIONS

    template<>
    class cvp_IplDepthTraits<char> {
    public:
        static const int depth = IPL_DEPTH_8S;
        static const int type = CV_8S;
    private:
        BOOST_STATIC_ASSERT(sizeof(char) == 1); // should always hold
    };

    template<>
    class cvp_IplDepthTraits<unsigned char> {
    public:
        static const int depth = IPL_DEPTH_8U;
        static const int type = CV_8U;
    private:
        BOOST_STATIC_ASSERT(sizeof(unsigned char) == 1); // should always hold
    };

    template<>
    class cvp_IplDepthTraits<short> {
    public:
        static const int depth = IPL_DEPTH_16S;
        static const int type = CV_16S;
    private:
        BOOST_STATIC_ASSERT(sizeof(short) == 2);
    };

    template<>
    class cvp_IplDepthTraits<unsigned short> {
    public:
        static const int depth = IPL_DEPTH_16U;
        static const int type = CV_16U;
    private:
        BOOST_STATIC_ASSERT(sizeof(unsigned short) == 2);
    };

    template<>
    class cvp_IplDepthTraits<int> {
    public:
        static const int depth = IPL_DEPTH_32S;
        static const int type = CV_32S;
    private:
        BOOST_STATIC_ASSERT(sizeof(int) == 4);
    };

    template<>
    class cvp_IplDepthTraits<float> {
    public:
        static const int depth = IPL_DEPTH_32F;
        static const int type = CV_32F;
    private:
        BOOST_STATIC_ASSERT(sizeof(float) == 4);
    };

    template<>
    class cvp_IplDepthTraits<double> {
    public:
        static const int depth = IPL_DEPTH_64F;
        static const int type = CV_64F;
    private:
        BOOST_STATIC_ASSERT(sizeof(double) == 8);
    };

    template<int depth_const>
    class cvp_IplTypeTraits {
    };

// SPECIALIZATIONS

    template<>
    class cvp_IplTypeTraits<IPL_DEPTH_8S> {
    public:
        typedef char type;
    private:
        BOOST_STATIC_ASSERT(sizeof(type) == 1); // should always hold
    };

    template<>
    class cvp_IplTypeTraits<IPL_DEPTH_8U> {
    public:
        typedef unsigned char type;
    private:
        BOOST_STATIC_ASSERT(sizeof(type) == 1); // should always hold
    };

    template<>
    class cvp_IplTypeTraits<IPL_DEPTH_16S> {
    public:
        typedef short type;
    private:
        BOOST_STATIC_ASSERT(sizeof(type) == 2);
    };

    template<>
    class cvp_IplTypeTraits<IPL_DEPTH_16U> {
    public:
        typedef unsigned short type;
    private:
        BOOST_STATIC_ASSERT(sizeof(type) == 2);
    };

    template<>
    class cvp_IplTypeTraits<IPL_DEPTH_32S> {
    public:
        typedef int type;
    private:
        BOOST_STATIC_ASSERT(sizeof(type) == 4);
    };

    template<>
    class cvp_IplTypeTraits<IPL_DEPTH_32F> {
    public:
        typedef float type;
    private:
        BOOST_STATIC_ASSERT(sizeof(type) == 4);
    };

    template<>
    class cvp_IplTypeTraits<IPL_DEPTH_64F> {
    public:
        typedef double type;
    private:
        BOOST_STATIC_ASSERT(sizeof(type) == 8);
    };


} // namespace OpenCvPlus

#endif // __CVP_IPLDEPTHTRAITS_H__
