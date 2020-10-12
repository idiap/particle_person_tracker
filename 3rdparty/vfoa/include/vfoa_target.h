/**
 * @file include/vfoa_target.h
 * @date 07 March 2013
 * @author Salim Kayal <salim.kayal@idiap.ch>
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Class to describe a target of visual focus of attention
 *
 * Copyright (C) 2011-2012 Idiap Research Institute, Martigny, Switzerland
 *
 */

#ifndef VFOA_TARGET_H
#define VFOA_TARGET_H

// SYSTEM INCLUDES
#include <fstream>                                     // STL ostream
#include <boost/math/distributions/normal.hpp>

// LOCAL INCLUDES
#include "types.h"
#include "utils.h"
//#include "person.h"

class Person;

//static const int UNFOCUSSED_OBJECT_ID = -1;
static const standard_deviation_t DEFAULT_STD(utils::PI/18,utils::PI/18,0);
//static const standard_deviation_t DEFAULT_UNFOCUSSED_STD(utils::PI/6,
//    utils::PI/6,0);
//static const float C_PAN = 1.5;
//static const float C_TILT = 1.5;
//static const float UNFOCUSED_PRIOR = (1/(2*utils::PI*DEFAULT_STD.pan))*
//    (1/(2*utils::PI*DEFAULT_STD.tilt))*(exp(1/2*((C_PAN/DEFAULT_STD.pan)+
//    (C_TILT/DEFAULT_STD.tilt))));
//static const float ALPHA_PAN(0.6);
//static const float ALPHA_TILT(0.4);
//static const float ALPHA_H_PAN(0);
//static const float ALPHA_H_TILT(0);

/// @brief Parameters of a target of visual focus of attention.
///
/// Parameters of a target of visual focus of attention that include standard
/// deviation to look at the target, reference factor that defines the
/// influence of reference direction of a person on person's gaze direction
/// to look at the target, and previous head pose factor that defines the
/// influence of person's previous gaze direction (head pose).
///
/// @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
/// @author Salim Kayal <salim.kayal@idiap.ch>
/// @version 1.0
/// @date   08.03.2013

struct VfoaTargetParameters {
    /// Standard deviation of a Gaussian distribution over gaze directions
    /// to look at the target
    standard_deviation_t m_StandardDeviation;
    /// Influence of reference direction of a person on person's gaze direction
    /// to look at the target
    pose_t m_ReferenceFactor;
    /// Influence of person's previous gaze direction on person's gaze direction
    /// to look at the target
    pose_t m_PreviousHeadPoseFactor;
}; // VfoaTargetParameters

std::ostream& operator<<(std::ostream& out,
        const VfoaTargetParameters &rParams);

/// @brief Represesntation of a target of visual focus of attention.
///
/// This class represents a target of visual focus of attention for some person,
/// who is called the owner of the target.
/// This target could be another person, an object in the scene or a special
/// value UNFOCUSED, meaning that currently there is no particular target.
/// For regular targets, distribution of person's gaze direction is calculated
/// that indicates which head poses of the person correspond more to the
/// situation when this person looks at the target.
///
/// @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
/// @author Salim Kayal <salim.kayal@idiap.ch>
/// @version 1.0
/// @date   04.03.2013

class VfoaTarget {

  public:
    /// Construct visual focus of attention target using its ID and type.
    /// @param rId Target's ID
    /// @param rType Target's type
    /// @param rLocation Target's location
    /// @param rOwner Target's owner
    /// @param rParams Target's parameters
    VfoaTarget(const target_id_t & rId, const type_t & rType,
        const location_t &rLocation, const Person & rOwner,
        const VfoaTargetParameters & rParams = DEFAULT_VFOA_TARGET_PARAMETERS);

    /// Construct visual focus of attention target from person's info.
    /// @param rPersonInfo Target person's info
    /// @param rOwner Target's owner
    /// @param rParams Target's parameters
    VfoaTarget(const person_info_t &rPersonInfo, const Person & rOwner,
        const VfoaTargetParameters & rParams = DEFAULT_VFOA_TARGET_PARAMETERS);

    /// Construct visual focus of attention target from object's info.
    /// @param rObjectInfo Target object's info
    /// @param rOwner Target's owner
    /// @param rParams Target's parameters
    VfoaTarget(const object_info_t &rObjectInfo, const Person & rOwner,
        const VfoaTargetParameters & rParams = DEFAULT_VFOA_TARGET_PARAMETERS);

    /// Construct unfocused visual focus of attention target.
    /// @param rOwner Target's owner
    /// @param rParams Target's parameters
    VfoaTarget(const Person & rOwner, const VfoaTargetParameters & rParams =
                DEFAULT_UNFOCUSED_TARGET_PARAMETERS);

    /// Constructs a copy of the target structure
    /// @param rTarget Target structure to copy
//    VfoaTarget(const VfoaTarget& rTarget);

    void UpdateTarget(const location_t &rLocation,
        const standard_deviation_t &rSigma = DEFAULT_STD);

    bool operator==(const VfoaTarget &rVfoaTarget);

    pose_t GetMean() const;
    pose_t GetGase() const;
    float GetPrior() const;

  public:

    /// Default parameters to be used by VFOA targets
    static const VfoaTargetParameters DEFAULT_VFOA_TARGET_PARAMETERS;

    /// Default parameters to be used by the UNFOCUSED target
    static const VfoaTargetParameters DEFAULT_UNFOCUSED_TARGET_PARAMETERS;

    const target_identifier_t cId;

  private:

    VfoaTarget& operator=(const VfoaTarget & rTarget);

    // output operator
    friend std::ostream& operator<<(std::ostream& out,
            const VfoaTarget &rVfoaTarget);

    // reference to the target's owner
    const Person& m_Owner;
    // VFOA target's parameters
    VfoaTargetParameters m_Params;
    // target's 3D location
    location_t m_Location;

    pose_t mMean;
    pose_t mGase;
    float mPrior;

    bool ComputeMean();
    void ComputeGase();
    void ComputePrior();

    void PrintToFile();

};

typedef std::list<VfoaTarget> vfoa_target_list_t;

#endif // VFOA_TARGET_H
