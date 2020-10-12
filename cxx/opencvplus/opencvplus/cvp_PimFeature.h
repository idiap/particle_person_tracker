/**
 * @file cxx/opencvplus/opencvplus/cvp_PimFeature.h
 * @date 25 May 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Probability index map (PIM) feature and PIM utilities
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_PIMFEATURE_H__
#define __CVP_PIMFEATURE_H__

// SYSTEM INCLUDES
#include <vector>                              // STL vector
#include <iostream>                            // STL streams
#include <fstream>                             // STL file stream
#include <boost/foreach.hpp>                   // foreach loop

// LOCAL INCLUDES
#include "opencvplus/cvp_Exceptions.h"         // various exceptions
#include "opencvplus/cvp_IplDepthTraits.h"     // maps types to OpenCV depth

// forward declaration of the class
namespace OpenCvPlus {
    template<typename T>
    class cvp_PimFeature;
}

// GLOBAL OPERATORS

/// Output operator.
///
/// @param os Output stream to put the histogram template
/// @param pf PIM to stream
/// @return A reference to the output stream.
template<typename T>
std::ostream& operator<<(std::ostream& os,
        const OpenCvPlus::cvp_PimFeature<T>& pim);

/// Input operator.
///
/// @param is Input stream to put the histogram template
/// @param pf PIM to stream
/// @return A reference to the input stream.
template<typename T>
std::istream& operator>>(std::istream& os,
        OpenCvPlus::cvp_PimFeature<T>& pim);


/// Substraction operator.
///
/// @param pf1 Minuend histogram template
/// @param pf2 Subtrahend histogram template
/// @return The elementwise difference of the two histogram templates.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator-(
        const OpenCvPlus::cvp_PimFeature<T>& pf1,
        const OpenCvPlus::cvp_PimFeature<T>& pf2);

/// Elementwise substraction of constant operator.
///
/// @param pf Minuend PIM feature
/// @param v  Subtrahent constant
/// @return The elementwise difference of the histogram template with a constant.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator-(
        const OpenCvPlus::cvp_PimFeature<T>& pf,
        const T& v);

/// Elementwise substraction of constant operator.
///
/// @param v  Minuend constant
/// @param pf Subtrahent PIM feature
/// @return The elementwise difference of the constant with the PIM feature.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator-(
        const T& v,
        const OpenCvPlus::cvp_PimFeature<T>& pf);

/// Addition operator.
///
/// @param pf1 Augend PIM feature
/// @param pf2 Addend PIM feature
/// @return The elementwise sum of the two PIM features.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator+(
        const OpenCvPlus::cvp_PimFeature<T>& pf1,
        const OpenCvPlus::cvp_PimFeature<T>& pf2);

/// Elementwise addition of constant operator.
///
/// @param pf Augend histogram template
/// @param v  Addend constant
/// @return The elementwise sum of the histogram template with a constant.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator+(
        const OpenCvPlus::cvp_PimFeature<T>& pf,
        const T& v);

/// Elementwise addition of constant operator.
///
/// @param v  Addend constant
/// @param pf Augend histogram template
/// @return The elementwise sum of the histogram template with a constant.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator+(
        const T& v,
        const OpenCvPlus::cvp_PimFeature<T>& pf);

/// Elementwise multiplication operator.
///
/// @param pf1 Multiplicand histogram template
/// @param pf2 Multiplier histogram template
/// @return The elementwise product of the two histogram templates.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator*(
        const OpenCvPlus::cvp_PimFeature<T>& pf1,
        const OpenCvPlus::cvp_PimFeature<T>& pf2);

/// Elementwise multiplication by a constant operator.
///
/// @param pf Multiplicand histogram template
/// @param v  Multiplier constant
/// @return The elementwise product of the histogram template by a constant.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator*(
        const OpenCvPlus::cvp_PimFeature<T>& pf,
        const T& v);

/// Elementwise multiplication by a constant operator.
///
/// @param v  Multiplier constant
/// @param pf Multiplicand histogram template
/// @return The elementwise product of the histogram template by a constant.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator*(
        const T& v,
        const OpenCvPlus::cvp_PimFeature<T>& pf);

/// Elementwise division operator.
///
/// @param pf1 Dividend histogram template
/// @param pf2 Divisor histogram template
/// @return The elementwise quotient of the two histogram templates.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator/(
        const OpenCvPlus::cvp_PimFeature<T>& pf1,
        const OpenCvPlus::cvp_PimFeature<T>& pf2);

/// Elementwise division by a constant operator.
///
/// @param pf Dividend histogram template
/// @param v  Divisor constant
/// @return The elementwise quotient of the histogram template by a constant.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator/(
        const OpenCvPlus::cvp_PimFeature<T>& pf,
        const T& v);

/// Elementwise division operator.
///
/// @param v  Dividend constant
/// @param pf Divisor histogram template
/// @return The elementwise quotient of a constant by the histogram template.
template<typename T>
OpenCvPlus::cvp_PimFeature<T> operator/(
        const T& v,
        const OpenCvPlus::cvp_PimFeature<T>& pf);

/// Elementwise equality operator.
///
/// @param pf1 Left-hands histogram template to compare
/// @param pf2 Rigpf-hands histogram template to compare
/// @return The elementwise equality of the two histogram templates.
template<typename T>
bool operator==(const OpenCvPlus::cvp_PimFeature<T>& pf1,
        const OpenCvPlus::cvp_PimFeature<T>& pf2);

namespace OpenCvPlus {

/// Probability index map (PIM) feature representation

template<typename T>
class cvp_PimFeature {

    public:

    // TYPES

    typedef T value_type;

    // LIFECYCLE

    /// Constructor
    /// @param num_histograms Number of channels in PIM
    /// @param width Width of each map
    /// @param height Height of each map
    cvp_PimFeature(unsigned num_channels, unsigned width, unsigned height) :
        m_Pim(num_channels), m_Width(width), m_Height(height) {
//        std::cout << "Constructing PIM feature " << this << std::endl;
        const std::vector<CvMat*>::iterator & first = m_Pim.begin();
        const std::vector<CvMat*>::iterator & last = m_Pim.end();
        for (std::vector<CvMat*>::iterator it = first; it != last; ++it) {
            *it = cvCreateMat(height, width, cvp_IplDepthTraits<T>::type);
            cvZero(*it);
//            std::cout << "Plane " << it - first
//                      << " : " << *it << "(pointer " << (*it)->data.fl << ")"
//                      << std::endl;
        }
    } // cvp_PimFeature

    /// Copy constructor
    /// @param other Other PIM to copy-construct from
    cvp_PimFeature(const cvp_PimFeature<T>& other) : m_Pim(other.channels()),
            m_Width(other.m_Width), m_Height(other.m_Height) {
//        std::cout << "Copy constructing PIM feature " << this << std::endl;
        const std::vector<CvMat*>::iterator & first = m_Pim.begin();
        const std::vector<CvMat*>::iterator & last = m_Pim.end();
        const std::vector<CvMat*>::const_iterator & ofirst = other.m_Pim.begin();
        const std::vector<CvMat*>::const_iterator & olast = other.m_Pim.end();
        std::vector<CvMat*>::iterator it;
        std::vector<CvMat*>::const_iterator oit;
        for (it = first, oit = ofirst; (it != last) && (oit != olast); ++it, ++oit) {
            *it = cvCloneMat(*oit);
//            std::cout << "Plane " << it - first
//                      << " : clone " << *oit << "(pointer " << (*oit)->data.fl << ")"
//                      << " to " << *it << "(pointer " << (*it)->data.fl << ")" << std::endl;
        }
    } // cvp_PimFeature

    /// Destructor
    ~cvp_PimFeature() {
//        std::cout << "Destructing PIM feature " << this << std::endl;
        const std::vector<CvMat*>::iterator & first = m_Pim.begin();
        const std::vector<CvMat*>::iterator & last = m_Pim.end();
        for (std::vector<CvMat*>::iterator it = first; it != last; ++it) {
//            std::cout << "Plane " << it - first
//                      << " : " << *it << "(pointer " << (*it)->data.fl << ")"
//                      << std::endl;
            cvReleaseMat(&(*it));
        }
    } // ~cvp_PimFeature


    // OPERATIONS

    /// Obtain a map by its channel for a read-only access
    /// @param channel Index of the channel
    /// @return Probability map for the given channel
    const CvMat* map(unsigned channel) const {
        if (channel >= m_Pim.size()) {
            throw cvp_Exception("PIM feature: requested channel number"
                " out of range");
        }
        return m_Pim[channel];
    }

    /// Obtain a map by its channel for a read/write access
    /// @param channel Index of the channel
    /// @return Probability map for the given channel
    CvMat* map(unsigned channel) {
        if (channel >= m_Pim.size()) {
            throw cvp_Exception("PIM feature: requested channel number"
                " out of range");
        }
        return m_Pim[channel];
    }

    /// Width of a map
    /// @return Width of a map
    unsigned width() const {
        return m_Width;
    }

    /// Height of a map
    /// @return Height of a map
    unsigned height() const {
        return m_Height;
    }

    /// Obtain the number of channels
    /// @return Number of channels in PIM
    unsigned channels() const {
        return m_Pim.size();
    }

    /// Reset all values in all maps to 0
    void clear() {
        BOOST_FOREACH(CvMat* mat, m_Pim) {
            cvSetZero(mat);
        }
    } // clear

    /// Normalise channels so that sum over all channels for a given pixel is 1
    void normalise_channels() {
        CvMat * temp = cvCreateMat(height(), width(), cvp_IplDepthTraits<T>::type);
        cvZero(temp);
        const std::vector<CvMat*>::iterator & first = m_Pim.begin();
        const std::vector<CvMat*>::iterator & last = m_Pim.end();
        for (std::vector<CvMat*>::iterator it = first; it != last; ++it) {
            cvAdd(*it, temp, temp);
        }
        for (std::vector<CvMat*>::iterator it = first; it != last; ++it) {
            cvDiv(*it, temp, *it);
        }
        cvReleaseMat(&temp);
    }

    /// Save PIM feature to a file.
    /// @param filename Name of a file to save the model to
    void save_to_file(const std::string& filename) const {

#define NUM_BYTES_INT 4
#define NUM_BYTES_DOUBLE 8

        std::ofstream pim_model_file;
        pim_model_file.open(filename.c_str(), std::ios::out | std::ios::binary);
        if (pim_model_file.is_open()) {
            try {
                std::ostringstream oss;
                typedef cvp_IplTypeTraits<IPL_DEPTH_32S>::type int_32;
                typedef cvp_IplTypeTraits<IPL_DEPTH_64F>::type double_64;

                char buffer_int[NUM_BYTES_INT]; // buffer for 32bit integers

                // write PIM feature size
                *(int_32*)buffer_int = static_cast<int_32>(m_Width);
                pim_model_file.write(buffer_int, NUM_BYTES_INT);
                *(int_32*)buffer_int = static_cast<int_32>(m_Height);
                pim_model_file.write(buffer_int, NUM_BYTES_INT);
                *(int_32*)buffer_int = static_cast<int_32>(m_Pim.size());
                pim_model_file.write(buffer_int, NUM_BYTES_INT);

                // write PIM feature data
                const unsigned pim_data_size = m_Width * m_Height * m_Pim.size();
                std::vector<double_64> pim_model_data(pim_data_size);
                const unsigned num_chars_to_write =
                        pim_data_size * NUM_BYTES_DOUBLE;
                to_array(pim_model_data);
                pim_model_file.write((char*)&(pim_model_data[0]), num_chars_to_write);
                pim_model_file.close();
            } catch(...) {
                pim_model_file.close();
                throw;
            }
        } else {
            throw cvp_Exception("PIM model file " + filename +
                " could not be opened for write!");
        }

#undef NUM_BYTES_DOUBLE
#undef NUM_BYTES_INT

    } // save_to_file

    /// Store PIM feature data into a plain array.
    /// @param arr Array to store PIM data
    template<typename U>
    void to_array(std::vector<U>& arr) const {
        assert(arr.size() == m_Width * m_Height * m_Pim.size());
        U * pArr = &arr[0];
        BOOST_FOREACH(CvMat * pPimDataMtx, m_Pim) {
            const T * pPimData = reinterpret_cast<T *>(pPimDataMtx->data.ptr);
            const unsigned pimDataStep = pPimDataMtx->step / sizeof(T) -
                m_Width;
            for (unsigned row = 0; row < m_Height; ++row) {
                for (unsigned col = 0; col < m_Width; ++col) {
                    *pArr++ = *pPimData++;
                }
                pPimData += pimDataStep;
            }
        }
    } // to_array

    /// Store PIM feature data into a plain array.
    /// @param arr Array to store PIM data
    template<typename U>
    void from_array(const std::vector<U>& arr) const {
        assert(arr.size() == m_Width * m_Height * m_Pim.size());
        const U * pArr = &arr[0];
        BOOST_FOREACH(CvMat * pPimDataMtx, m_Pim) {
            T * pPimData = reinterpret_cast<T*>(pPimDataMtx->data.ptr);
            const unsigned pimDataStep = pPimDataMtx->step / sizeof(T) -
                m_Width;
            for (unsigned row = 0; row < m_Height; ++row) {
                for (unsigned col = 0; col < m_Width; ++col) {
                    *pPimData++ = *pArr++;
                }
                pPimData += pimDataStep;
            }
        }
    } // from_array

    // OPERATORS

    /// Assignment operator.
    ///
    /// @param rhs PIM to assign
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator=(const cvp_PimFeature<T>& rhs) {
//        std::cout << "Assigning PIM feature " << this << " = " << &rhs << std::endl;
        if (this != &rhs) {
            if (m_Pim.size() != rhs.channels()) {
                throw cvp_Exception("PIM feature assignment performed on "
                    "features with different channels number");
            } else if ((m_Pim.size() > 0) && ((width() != rhs.width()) ||
                       (height() != rhs.height()))) {
                throw cvp_Exception("PIM feature assignment performed on "
                    "features of different size");
            }
            const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
            const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
            std::vector<CvMat*>::iterator mit;

            const std::vector<CvMat*>::const_iterator & rfirst = rhs.m_Pim.begin();
            const std::vector<CvMat*>::const_iterator & rlast = rhs.m_Pim.end();
            std::vector<CvMat*>::const_iterator rit;

            for (mit = mfirst, rit = rfirst; (mit != mlast) && (rit != rlast);
                ++mit, ++rit) {
//                std::cout << "Plane " << mit - mfirst
//                          << " : clone " << *rit << "(pointer " << (*rit)->data.fl << ")"
//                          << " to " << *mit << "(pointer " << (*mit)->data.fl << ")" << std::endl;
                cvCopy(*rit, *mit);
            }
        }
        return *this;
    } // =

    /// Assignment operator.
    ///
    /// @param rhs Scalar value to assign
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator=(const T& rhs) {
        const std::vector<CvMat*>::const_iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::const_iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::const_iterator mit;
        for (mit = mfirst; mit != mlast; ++mit) {
            cvSet(*mit, cvScalar(rhs));
        }
        return *this;
    } // =

    /// Addition assignment operator.
    ///
    /// @param rhs PIM to add
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator+=(const cvp_PimFeature<T>& rhs) {
        if (m_Pim.size() != rhs.channels()) {
            throw cvp_Exception("PIM feature assignment performed on features "
                "with different channels number");
        }
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        const std::vector<CvMat*>::const_iterator & rfirst = rhs.m_Pim.begin();
        const std::vector<CvMat*>::const_iterator & rlast = rhs.m_Pim.end();
        std::vector<CvMat*>::const_iterator rit;

        for (mit = mfirst, rit = rfirst; (mit != mlast) && (rit != rlast);
            ++mit, ++rit) {
            cvAdd(*rit, *mit, *mit);
        }
        return *this;
    } // +=

    /// Constant addition assignment operator.
    ///
    /// @param rhs Constant to add
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator+=(const T& rhs) {
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        for (mit = mfirst; mit != mlast; ++mit) {
            cvAddS(*mit, cvScalar(rhs), *mit);
        }
        return *this;
    } // +=

    /// Substraction assignment operator.
    ///
    /// @param rhs PIM to substract
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator-=(const cvp_PimFeature<T>& rhs) {
        if (m_Pim.size() != rhs.channels()) {
            throw cvp_Exception("PIM feature assignment performed on features "
                "with different channels number");
        }
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        const std::vector<CvMat*>::const_iterator & rfirst = rhs.m_Pim.begin();
        const std::vector<CvMat*>::const_iterator & rlast = rhs.m_Pim.end();
        std::vector<CvMat*>::const_iterator rit;

        for (mit = mfirst, rit = rfirst; (mit != mlast) && (rit != rlast);
            ++mit, ++rit) {
            cvSub(*mit, *rit, *mit);
        }
        return *this;
    } // -=

    /// Constant substraction assignment operator.
    ///
    /// @param rhs Constant to substract
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator-=(const T& rhs) {
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        for (mit = mfirst; mit != mlast; ++mit) {
            cvSubS(*mit, cvScalar(rhs), *mit);
        }
        return *this;
    } // -=

    /// Multiplication assignment operator.
    ///
    /// @param rhs PIM to multiply with
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator*=(const cvp_PimFeature<T>& rhs) {
        if (m_Pim.size() != rhs.channels()) {
            throw cvp_Exception("PIM feature assignment performed on features "
                "with different channels number");
        }
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        const std::vector<CvMat*>::const_iterator & rfirst = rhs.m_Pim.begin();
        const std::vector<CvMat*>::const_iterator & rlast = rhs.m_Pim.end();
        std::vector<CvMat*>::const_iterator rit;

        for (mit = mfirst, rit = rfirst; (mit != mlast) && (rit != rlast);
            ++mit, ++rit) {
            cvMul(*rit, *mit, *mit);
        }
        return *this;
    } // *=

    /// Constant multiplication assignment operator.
    ///
    /// @param rhs Constant to multiply with
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator*=(const T& rhs) {
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        for (mit = mfirst; mit != mlast; ++mit) {
            cvScale(*mit, *mit, rhs);
        }
        return *this;
    } // *=

    /// Division assignment operator.
    ///
    /// @param rhs Head pose structure to divide with
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T>& operator/=(const cvp_PimFeature<T>& rhs) {
        if (m_Pim.size() != rhs.channels()) {
            throw cvp_Exception("PIM feature assignment performed on features "
                "with different channels number");
        }
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        const std::vector<CvMat*>::const_iterator & rfirst = rhs.m_Pim.begin();
        const std::vector<CvMat*>::const_iterator & rlast = rhs.m_Pim.end();
        std::vector<CvMat*>::const_iterator rit;

        for (mit = mfirst, rit = rfirst; (mit != mlast) && (rit != rlast);
            ++mit, ++rit) {
            cvDiv(*mit, *rit, *mit);
        }
        return *this;
    } // /=

    /// Constant division assignment operator.
    ///
    /// @param rhs Constant to divide with
    /// @return Reference to self with modified contents.
    cvp_PimFeature<T> operator/=(const T& rhs) {
        if (0 == rhs) {
            throw cvp_Exception("PIM feature divide on 0");
        }
        const std::vector<CvMat*>::iterator & mfirst = m_Pim.begin();
        const std::vector<CvMat*>::iterator & mlast = m_Pim.end();
        std::vector<CvMat*>::iterator mit;

        for (mit = mfirst; mit != mlast; ++mit) {
            cvScale(*mit, *mit, 1.0 / rhs);
        }
        return *this;
    } // /=

    private:

    // vector of probability maps, all elements (matrices) are of the same size
    std::vector<CvMat*> m_Pim;
    unsigned m_Width;  // width of a matrix
    unsigned m_Height; // height of a matrix

    // FRIEND DECLARATIONS

    template<typename U>
    friend std::ostream& (::operator<<)(std::ostream& os,
            const cvp_PimFeature<U>& pim);

    template<typename U>
    friend std::istream& (::operator>>)(std::istream& os,
            cvp_PimFeature<U>& pim);

    template<typename U>
    friend cvp_PimFeature<U> (::operator-)(const cvp_PimFeature<U>& pf1,
            const cvp_PimFeature<U>& pf2);

    template<typename U>
    friend cvp_PimFeature<U> (::operator-)(const cvp_PimFeature<U>& pf,
            const U& v);

    template<typename U>
    friend cvp_PimFeature<U> (::operator-)(const U& v,
            const cvp_PimFeature<U>& pf);

    template<typename U>
    friend cvp_PimFeature<U> (::operator+)(const cvp_PimFeature<U>& pf1,
            const cvp_PimFeature<U>& pf2);

    template<typename U>
    friend cvp_PimFeature<U> (::operator+)(const cvp_PimFeature<U>& pf,
            const U& v);

    template<typename U>
    friend cvp_PimFeature<U> (::operator+)(const U& v,
            const cvp_PimFeature<U>& pf);

    template<typename U>
    friend cvp_PimFeature<U> (::operator*)(const cvp_PimFeature<U>& pf1,
            const cvp_PimFeature<U>& pf2);

    template<typename U>
    friend cvp_PimFeature<U> (::operator*)(const cvp_PimFeature<U>& pf,
            const U& v);

    template<typename U>
    friend cvp_PimFeature<U> (::operator*)(const U& v,
            const cvp_PimFeature<U>& pf);

    template<typename U>
    friend cvp_PimFeature<U> (::operator/)(const cvp_PimFeature<U>& pf1,
            const cvp_PimFeature<U>& pf2);

    template<typename U>
    friend cvp_PimFeature<U> (::operator/)(const cvp_PimFeature<U>& pf,
            const U& v);

    template<typename U>
    friend cvp_PimFeature<U> (::operator/)(const U& v,
            const cvp_PimFeature<U>& pf);

    template<typename U>
    friend U distance_correlation_global(const cvp_PimFeature<U>& pim1,
           const cvp_PimFeature<U>& pim2);

    template<typename U>
    friend U distance_correlation_pixelwise(const cvp_PimFeature<U>& pim1,
            const cvp_PimFeature<U>& pim2);

};

template<typename T>
T distance_correlation_pixelwise(const cvp_PimFeature<T>& pim1,
        const cvp_PimFeature<T>& pim2) {

    if (pim1.channels() != pim2.channels()) {
        throw cvp_Exception("PIM feature correlation measure applied to "
            "features with different channels number");
    }

    int height = pim1.m_Pim[0]->height;
    int width = pim1.m_Pim[0]->width;
    CvMat * corr = cvCreateMat(height, width, cvp_IplDepthTraits<T>::type);
    CvMat * norm1 = cvCreateMat(height, width, cvp_IplDepthTraits<T>::type);
    CvMat * norm2 = cvCreateMat(height, width, cvp_IplDepthTraits<T>::type);
    CvMat * temp = cvCreateMat(height, width, cvp_IplDepthTraits<T>::type);

    cvSetZero(corr);
    cvSetZero(norm1);
    cvSetZero(norm2);

    const std::vector<CvMat*>::const_iterator & first1 = pim1.m_Pim.begin();
    const std::vector<CvMat*>::const_iterator & last1 = pim1.m_Pim.end();
    std::vector<CvMat*>::const_iterator it1;

    const std::vector<CvMat*>::const_iterator first2 = pim2.m_Pim.begin();
    const std::vector<CvMat*>::const_iterator last2 = pim2.m_Pim.end();
    std::vector<CvMat*>::const_iterator it2;

    for (it1 = first1, it2 = first2; (it1 != last1) && (it2 != last2);
        ++it1, ++it2) {
        cvMul(*it1, *it2, temp);
        cvAdd(corr, temp, corr);
        cvMul(*it1, *it1, temp);
        cvAdd(norm1, temp, norm1);
        cvMul(*it2, *it2, temp);
        cvAdd(norm2, temp, norm2);
    }

    cvPow(norm1, norm1, 0.5);
    cvPow(norm2, norm2, 0.5);
    cvDiv(corr, norm1, corr);
    cvDiv(corr, norm2, corr);

    float result = cvSum(corr).val[0] / (width * height);

    // TODO: add try ... finally statement (BOOST)
    cvReleaseMat(&temp);
    cvReleaseMat(&norm2);
    cvReleaseMat(&norm1);
    cvReleaseMat(&corr);

    return result;

} // distance_correlation_pixelwise

template<typename T>
T distance_correlation_global(const cvp_PimFeature<T>& pim1,
        const cvp_PimFeature<T>& pim2) {
    if (pim1.channels() != pim2.channels()) {
        throw cvp_Exception("PIM feature correlation measure applied to "
            "features with different channels number");
    }

    int height = pim1.m_Pim[0]->height;
    int width = pim1.m_Pim[0]->width;
    CvMat * temp = cvCreateMat(height, width, cvp_IplDepthTraits<T>::type);

    const std::vector<CvMat*>::const_iterator & first1 = pim1.m_Pim.begin();
    const std::vector<CvMat*>::const_iterator & last1 = pim1.m_Pim.end();
    std::vector<CvMat*>::const_iterator it1;

    const std::vector<CvMat*>::const_iterator & first2 = pim2.m_Pim.begin();
    const std::vector<CvMat*>::const_iterator & last2 = pim2.m_Pim.end();
    std::vector<CvMat*>::const_iterator it2;

    float corr = 0;
    float norm1 = 0;
    float norm2 = 0;

    for (it1 = first1, it2 = first2; (it1 != last1) && (it2 != last2);
        ++it1, ++it2) {
        cvMul(*it1, *it2, temp);
        corr += cvSum(temp).val[0];
        cvMul(*it1, *it1, temp);
        norm1 += cvSum(temp).val[0];
        cvMul(*it2, *it2, temp);
        norm2 += cvSum(temp).val[0];
    }

    float result = corr / sqrt(norm1) / sqrt(norm2);

    // TODO: add try ... finally statement (BOOST)
    cvReleaseMat(&temp);

    return result;

} // distance_correlation_global

} // namespace OpenCvPlus

// GLOBAL OPERATORS DEFINITION

template<typename T>
bool operator==(const OpenCvPlus::cvp_PimFeature<T>& pf1,
        const OpenCvPlus::cvp_PimFeature<T>& pf2) {

    if ((pf1.channels() != pf2.channels()) || (pf1.width() != pf2.width()) ||
        (pf1.height() != pf2.height())) {
        return false;
    }

    if (pf1.channels() > 0) {

        CvMat * cmp_res = cvCreateMat(pf1.height(), pf1.width(), CV_8UC1);

        for (unsigned channel = 0; channel < pf1.channels(); ++channel) {
            cvCmp(pf1.map(channel), pf2.map(channel), cmp_res, CV_CMP_NE);
            if (cvCountNonZero(cmp_res)) {
                return false;
            }
        }

        cvReleaseMat(&cmp_res);
    }

    return true;

}

template<typename T>
bool operator!=(const OpenCvPlus::cvp_PimFeature<T>& pf1,
        const OpenCvPlus::cvp_PimFeature<T>& pf2) {

    return !(pf1 == pf2);

}

#endif // __CVP_PIMFEATURE_H__
