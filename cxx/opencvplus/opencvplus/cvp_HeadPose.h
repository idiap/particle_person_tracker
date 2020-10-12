// Copyright (c) 2010-2020 Idiap Research Institute
//
// cvp_HeadPose - class to represent a pan/tilt/roll parametrization
//                 of a head pose
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __CVP_HEADPOSE_H__
#define __CVP_HEADPOSE_H__

// SYSTEM INCLUDES
#include <iostream>                       // STL IO streaming
#include <boost/serialization/access.hpp> // private access for serialisation
#include <boost/serialization/nvp.hpp>    // for make_nvp

namespace OpenCvPlus {

/// @brief Class to represent a pan/tilt/roll parametrization of a head pose
///
/// This class defines a simple parametrization of a head pose through
/// three angles: pan and tilt (out-of-plane rotations) and roll
/// (in-plane rotation). The angles are computed with respect to some reference
/// position which is taken to be 0.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    19.01.2011

    template<typename RealType = float>
    class cvp_HeadPose {

        public:

        // TYPES

        typedef RealType value_type;

        // LIFECYCLE

        /// Default constructor, all angles are set to 0
        cvp_HeadPose() : mPan(0), mTilt(0), mRoll(0) {}

        /// Convertion constructor for floats, all angles are set to the
        /// provided value
        /// @param v The value to assign to all the angles
        cvp_HeadPose(RealType v) : mPan(v), mTilt(v), mRoll(v) {}

        /// Constructor from the angle values
        /// @param pan The initial value for the pan angle
        /// @param tilt The initial value for the tilt angle
        /// @param roll The initial value for the roll angle
        cvp_HeadPose(RealType pan, RealType tilt, RealType roll) :
            mPan(pan), mTilt(tilt), mRoll(roll) {}

        /// Copy constructor
        /// @param hp The head pose to copy from
        template<typename T>
        cvp_HeadPose(const cvp_HeadPose<T>& hp) : mPan(hp.mPan),
            mTilt(hp.mTilt), mRoll(hp.mRoll) {}

        // OPERATIONS

        /// Set pan rotation angle value
        /// @param p The new value of the pan rotation angle
        void pan(const RealType& p) { mPan = p; }

        /// Get pan rotation angle value
        /// @return The value of the pan rotation angle
        RealType pan() const { return mPan; }

        /// Set tilt rotation angle value
        /// @param t The new value of the tilt rotation angle
        void tilt(const RealType& t) { mTilt = t; }

        /// Get tilt rotation angle value
        /// @return The value of the tilt rotation angle
        RealType tilt() const { return mTilt; }

        /// Set roll rotation angle value
        /// @param r The new value of the roll rotation angle
        void roll(const RealType& r) { mRoll = r; }

        /// Get roll rotation angle value
        /// @return The value of the roll rotation angle
        RealType roll() const { return mRoll; }

        // OPERATORS

        /// Assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1, head_pose_2;
        /// ...
        /// head_pose_1 = head_pose_2;
        ///
        /// @param rhs Head pose structure to assign
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator=(const cvp_HeadPose<T>& rhs) {
            if (this != &rhs) {
                mPan = rhs.pan();
                mTilt = rhs.tilt();
                mRoll = rhs.roll();
            }
            return *this;
        }

        /// Addition assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1, head_pose_2;
        /// ...
        /// head_pose_1 += head_pose_2;
        ///
        /// @param rhs Head pose structure to add
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator+=(const cvp_HeadPose<T>& rhs) {
            mPan += rhs.pan();
            mTilt += rhs.tilt();
            mRoll += rhs.roll();
            return *this;
        }

        /// Addition assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1;
        /// float v;
        /// ...
        /// head_pose_1 += v;
        ///
        /// @param rhs Head pose structure to add
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator+=(const T& rhs) {
            mPan += rhs;
            mTilt += rhs;
            mRoll += rhs;
            return *this;
        }


        /// Substraction assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1, head_pose_2;
        /// ...
        /// head_pose_1 -= head_pose_2;
        ///
        /// @param rhs Head pose structure to substract
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator-=(const cvp_HeadPose<T>& rhs) {
            mPan -= rhs.pan();
            mTilt -= rhs.tilt();
            mRoll -= rhs.roll();
            return *this;
        }

        /// Substraction assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1;
        /// float v;
        /// ...
        /// head_pose_1 -= v;
        ///
        /// @param rhs Head pose structure to substract
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator-=(const T& rhs) {
            mPan -= rhs;
            mTilt -= rhs;
            mRoll -= rhs;
            return *this;
        }

        /// Multiplication assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1, head_pose_2;
        /// ...
        /// head_pose_1 *= head_pose_2;
        ///
        /// @param rhs Head pose structure to multiply with
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator*=(const cvp_HeadPose<T>& rhs) {
            mPan *= rhs.pan();
            mTilt *= rhs.tilt();
            mRoll *= rhs.roll();
            return *this;
        }

        /// Multiplication assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1;
        /// float v;
        /// ...
        /// head_pose_1 *= v;
        ///
        /// @param rhs Head pose structure to multiply with
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator*=(const T& rhs) {
            mPan *= rhs;
            mTilt *= rhs;
            mRoll *= rhs;
            return *this;
        }

        /// Division assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1, head_pose_2;
        /// ...
        /// head_pose_1 /= head_pose_2;
        ///
        /// @param rhs Head pose structure to divide with
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator/=(const cvp_HeadPose<T>& rhs) {
            mPan /= rhs.pan();
            mTilt /= rhs.tilt();
            mRoll /= rhs.roll();
            return *this;
        }

        /// Division assignment operator.
        ///
        /// EXAMPLES
        ///
        /// vfoa_HeadPose head_pose_1;
        /// float v;
        /// ...
        /// head_pose_1 /= v;
        ///
        /// @param rhs Head pose structure to divide with
        /// @return Reference to self with modified contents.
        template<typename T>
        cvp_HeadPose<RealType>& operator/=(const T& rhs) {
            mPan /= rhs;
            mTilt /= rhs;
            mRoll /= rhs;
            return *this;
        }

        private:

        // give access to private fields to serialize properly
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int ver) {
            ar & boost::serialization::make_nvp("pan", mPan);
            ar & boost::serialization::make_nvp("tilt", mTilt);
            ar & boost::serialization::make_nvp("roll", mRoll);
        }

        RealType mPan;  // pan rotation angle
        RealType mTilt; // tilt rotation angle
        RealType mRoll; // roll rotation angle

    };

// GLOBAL OPERATORS

/// Output operator.
///
/// EXAMPLE
///
/// cvp_HeadPose head_pose;
/// ...
/// cout << head_pose;
///
/// @param os Output stream to put the head pose structure
/// @param hp Head pose structure to stream
/// @return A reference to the output stream.
template<typename T>
std::ostream& operator<<(std::ostream& os,
    const OpenCvPlus::cvp_HeadPose<T>& hp) {
    os << hp.pan() << ' ' << hp.tilt() << ' ' << hp.roll();
    return os;
}

/// Input operator.
///
/// EXAMPLE
///
/// cvp_HeadPose head_pose;
/// ...
/// cin >> head_pose;
///
/// @param is Input stream to put the head pose structure
/// @param hp Head pose structure to stream
/// @return A reference to the input stream.
template<typename T>
std::istream& operator>>(std::istream& is, OpenCvPlus::cvp_HeadPose<T>& hp) {
    try {
        T v;
        is >> v; hp.pan(v);
        is >> v; hp.tilt(v);
        is >> v; hp.roll(v);
    } catch(...) {
        std::cerr << "Error reading head pose structure from a stream!" <<
            std::endl;
        throw;
    }
    return is;
}

/// Substraction operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2;
/// ...
/// head_pose_diff = head_pose_1 - head_pose_2;
/// head_pose_diff = head_pose_1 - 1.0;
///
/// @param hp1 Minuend head pose structure
/// @param hp2 Subtrahend head pose structure
/// @return The elementwise difference of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator-(
    const OpenCvPlus::cvp_HeadPose<T1>& hp1,
    const OpenCvPlus::cvp_HeadPose<T2>& hp2) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp1.pan() - hp2.pan(),
        hp1.tilt() - hp2.tilt(), hp1.roll() - hp2.roll());
}

/// Substraction operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2;
/// ...
/// head_pose_diff = head_pose_1 - head_pose_2;
/// head_pose_diff = head_pose_1 - 1.0;
///
/// @param hp1 Minuend head pose structure
/// @param hp2 Subtrahend head pose structure
/// @return The elementwise difference of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator-(
    const OpenCvPlus::cvp_HeadPose<T1>& hp,
    const T2& v) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp.pan() - v,
        hp.tilt() - v, hp.roll() - v);
}

/// Addition operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2, head_pose_sum;
/// ...
/// head_pose_sum = head_pose_1 + head_pose_2;
/// head_pose_sum = head_pose_1 + 2.0;
///
/// @param hp1 Augend head pose structure
/// @param hp2 Addend head pose structure
/// @return The elementwise sum of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator+(
    const OpenCvPlus::cvp_HeadPose<T1>& hp1,
    const OpenCvPlus::cvp_HeadPose<T2>& hp2) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp1.pan() + hp2.pan(),
        hp1.tilt() + hp2.tilt(), hp1.roll() + hp2.roll());
}

/// Addition operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2, head_pose_sum;
/// ...
/// head_pose_sum = head_pose_1 + head_pose_2;
/// head_pose_sum = head_pose_1 + 2.0;
///
/// @param hp1 Augend head pose structure
/// @param hp2 Addend head pose structure
/// @return The elementwise sum of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator+(
    const OpenCvPlus::cvp_HeadPose<T1>& hp,
    const T2& v) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp.pan() + v,
        hp.tilt() + v, hp.roll() + v);
}


/// Elementwise multiplication operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2, head_pose_mul;
/// ...
/// head_pose_mul = head_pose_1 * head_pose_2;
/// head_pose_mul = head_pose_1 * 2.0;
///
/// @param hp1 Multiplicand head pose structure
/// @param hp2 Multiplier head pose structure
/// @return The elementwise product of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator*(
    const OpenCvPlus::cvp_HeadPose<T1>& hp1,
    const OpenCvPlus::cvp_HeadPose<T2>& hp2) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp1.pan() * hp2.pan(),
        hp1.tilt() * hp2.tilt(), hp1.roll() * hp2.roll());
}

/// Elementwise multiplication operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2, head_pose_mul;
/// ...
/// head_pose_mul = head_pose_1 * head_pose_2;
/// head_pose_mul = head_pose_1 * 2.0;
///
/// @param hp1 Multiplicand head pose structure
/// @param hp2 Multiplier head pose structure
/// @return The elementwise product of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator*(
    const OpenCvPlus::cvp_HeadPose<T1>& hp,
    const T2& v) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp.pan() * v, hp.tilt() * v,
        hp.roll() * v);
}


/// Elementwise division operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2, head_pose_div;
/// ...
/// head_pose_div = head_pose_1 / head_pose_2;
/// head_pose_div = head_pose_1 / 2.0;
///
/// @param hp1 Dividend head pose structure
/// @param hp2 Divisor head pose structure
/// @return The elementwise quotient of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator/(
    const OpenCvPlus::cvp_HeadPose<T1>& hp1,
    const OpenCvPlus::cvp_HeadPose<T2>& hp2) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp1.pan() / hp2.pan(),
        hp1.tilt() / hp2.tilt(), hp1.roll() / hp2.roll());
}

/// Elementwise division operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2, head_pose_div;
/// ...
/// head_pose_div = head_pose_1 / head_pose_2;
/// head_pose_div = head_pose_1 / 2.0;
///
/// @param hp1 Dividend head pose structure
/// @param hp2 Divisor head pose structure
/// @return The elementwise quotient of the two head pose structures.
template<typename T1, typename T2>
OpenCvPlus::cvp_HeadPose<T1> operator/(
    const OpenCvPlus::cvp_HeadPose<T1>& hp,
    const T2& v) {
    return OpenCvPlus::cvp_HeadPose<T1>(hp.pan() / v,
        hp.tilt() / v, hp.roll() / v);
}

/// Elementwise equality operator.
///
/// EXAMPLES
/// The operator can be applied to any arguments that can be implicitly
/// converted to head pose structure.
///
/// cvp_HeadPose head_pose_1, head_pose_2;
/// ...
/// if (head_pose_1 == head_pose_2) {
///     ...
/// }
///
/// @param hp1 Left-hands head pose structure to compare
/// @param hp2 Right-hands head pose structure to compare
/// @return The elementwise equality of the two head pose structures.
template<typename T1, typename T2>
bool operator==(const OpenCvPlus::cvp_HeadPose<T1>& hp1,
    const OpenCvPlus::cvp_HeadPose<T2>& hp2) {
    return (hp1.pan() == hp2.pan()) && (hp1.tilt() == hp2.tilt()) &&
        (hp1.roll() == hp2.roll());
}


/// Elementwise absolute value of the head pose.
///
/// EXAMPLE
///
/// cvp_HeadPose head_pose, head_pose_abs;
/// ...
/// head_pose_abs = abs(head_pose);
///
/// @param hp Head pose to compute the elementwise absolute value for
/// @return The head pose structure containing elementwise absolute values.
template<typename T>
OpenCvPlus::cvp_HeadPose<T> abs(const OpenCvPlus::cvp_HeadPose<T>& hp) {
    return OpenCvPlus::cvp_HeadPose<T>(fabs(hp.pan()), fabs(hp.tilt()),
        fabs(hp.roll()));
}

/// L-infinity distance between head poses, equal to the maximum of
/// absolute values of coordinate-wise differences between head poses
///
/// @param hp1 First head pose
/// @param hp2 Second head pose
/// @return L-infinity distance between head poses.
template<typename T>
T distance_Linf(const OpenCvPlus::cvp_HeadPose<T>& hp1,
    const OpenCvPlus::cvp_HeadPose<T>& hp2) {
    return std::max(
        std::max(fabs(hp1.pan() - hp2.pan()), fabs(hp1.tilt() - hp2.tilt())),
        fabs(hp1.roll() - hp2.roll()));
}

} // namespace OpenCvPlus

#endif // __CVP_HEADPOSE_H__
