#ifndef TYPES_H
#define TYPES_H

/**
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include <string>
#include <list>
#include <iostream>
#include "boost/date_time/posix_time/posix_time.hpp"

///target type enumerator
enum type_t{
  ERROR = 0, ///<Error, target cannot be identified
  OBJECT_INFO, ///<Target is an object
  PERSON_INFO, ///<Target is a person
  UNFOCUSSED_VALUE};///<target is unfocussed

///Overloaded operator << for ostream output
inline std::ostream& operator<<(std::ostream& out, const type_t &rType){
  switch (rType){
    case ERROR:
      out<<"ERROR";
      break;
    case OBJECT_INFO:
      out<<"OBJECT_INFO";
      break;
    case PERSON_INFO:
      out<<"PERSON_INFO";
      break;
    case UNFOCUSSED_VALUE:
      out<<"UNFOCUSSED_VALUE";
  }
  return out;
}

///target identification number
typedef int target_id_t;

///Pose structure
/**Pose structure using angles in radians
 * (pan = 0, tilt = 0, roll = 0) means camera direction
 */
struct pose_t {

  float pan;///<pan angle
  float tilt;///<tilt angle
  float roll;///<roll angle

  ///default constructor
  pose_t();

  ///contructor
  /**constructor using angles
   * \param rPan pan angle
   * \param rTilt tilt angle
   * \param rRoll roll angle
   */
  pose_t(float rPan, float rTilt, float rRoll);

  ///copy constructor
  /**\param rPose pose_t to copy
  */
  pose_t(const pose_t &rPose);

  ///overloaded operator +
  /**adds an other pose_t to itself and returns the result
   * \param rAddPose pose to add
   * \return addition result
   */
  pose_t operator +(pose_t rAddPose);

  ///overloaded operator +=
  /**adds an other pose_t to self and reassign its own values
   * \param rAddPose pose to add
   */
  void operator +=(pose_t rAddPose);

  ///overloaded operator *
  /**multiplies self with a float and returns the result
   * \param rMultiplier multiplier
   * \return multiplication result
   */
  pose_t operator *(float rMultiplier);

  ///overloaded operator /
  /**divides self with an other pose_t and returns the result
   * \param rDivider divider pose
   * \return division result
   */
  pose_t operator /(float rDivider);

  ///overloaded operator +=
  /**divides self with an other pose_t and updates its values
   * \param rDivider divider pose
   */
  void operator /= (float rDivider);

  ///overloaded operator =
  /**copies the content of an other pose_t in itself
   * \param rEqualPose value to assign
   */
  pose_t& operator = (pose_t rEqualPose);

  ///overloaded ostream operator<< for pose_t
  friend std::ostream& operator<<(std::ostream& out, const pose_t &rPose);
};

/// standard_deviation type for pan tilt roll
typedef pose_t standard_deviation_t;

///timestamped pose
struct timestamped_pose_t{
  pose_t pose;
  boost::posix_time::ptime timestamp;
  timestamped_pose_t(const pose_t & rPose,
      const boost::posix_time::ptime &rTimestamp);
  timestamped_pose_t(const timestamped_pose_t &rTimestampedPose);
  timestamped_pose_t& operator=(const timestamped_pose_t &rTimestampedPose);
};

///Location structure
/**location structure used to define locations in 3d positions
*/
struct location_t {

  float x;///<x position
  float y;///<y position
  float z;///<z position

  ///constructor
  /**default constructor
  */
  location_t();

  ///constructor
  /**constructor
   * \param rX x value
   * \param rY y value
   * \param rZ zvalue
   */
  location_t(const float &rX, const float &rY, const float &rZ);

  ///copy constructor
  /**copy constructor
   * \param rLocation location_t to copy
   */
  location_t(const location_t &rLocation);

  ///overloaded operator -
  /**substract a location to an other
   * \param rLocation location to substract
   * \return result of the substraction
   */
  location_t operator - (location_t rLocation);

  ///overloaded operator /
  /**divide a location with an other
   * \param rDivider divisor
   * \return result of the division
   */
  location_t operator / (float rDivisor);


  void operator /= (float rDivisor);

///overloaded operator =
  /**assign a location to an other
   * \param rLocation location to assign
   * \return reference to self
   */
  location_t& operator = (const location_t &rLocation);

  ///overloaded ostream operator << for location_t
  friend std::ostream& operator<<(std::ostream& out, const location_t &rLoc);
};

///same functions than location_t but other meaning
typedef location_t vector_3d_t;

///Basis using three vector_3d_t
typedef struct  basis_t{vector_3d_t axis[3];} basis_t;
std::ostream& operator<<(std::ostream& out, const basis_t & rBasis);

///object info structure
/**contains all the information regarding an object.
 * location must be given using (0,0,0) as the camera position
 */
struct object_info_t{

  target_id_t id;///<identification number of the object
  std::string name;///<name of the object
  location_t loc;///<location of the object. (0,0,0) is the camera

  ///default constructor
  object_info_t();

  ///copy constructor
  /**\param rObjectInfo object to copy
  */
  object_info_t(const object_info_t &rObjectInfo);

  ///overloaded ostream operator << for object_info_t
  friend std::ostream& operator<<(std::ostream& out,
      const object_info_t &rObjectInfo);
};

///list of objects
typedef std::list<object_info_t> object_list_t;
std::ostream& operator<<(std::ostream& out, const object_list_t & rOlist);

///person info structure
/**contains all the information regarding a person.
 * location must be given using (0,0,0) as the camera position
 * head pose (0,0,0) is the person facing directly the camera
 */
struct person_info_t{
  target_id_t id;///<identification number of the person
  std::string name;///<name of the person
  location_t loc;///<location of the person. (0,0,0) is the camera
  ///head pose of the person. (0,0,0) is the person facing the camera
  pose_t head_pose;

  ///constructor
  person_info_t();

  ///copy constructor
  /**\param rPersonInfo person_info_t to copy
  */
  person_info_t(const person_info_t &rPersonInfo);

  ///overloaded operator =
  /**assignment operator for person_info_t
   * \param rPersonInfo value to assign
   * \return *this
   */
  person_info_t& operator = (person_info_t rPersonInfo);

  ///overloaded ostream operator << for person_info_t
  friend std::ostream& operator<<(std::ostream& out,
          const person_info_t & rPersonInfo);
};

///list of persons
typedef std::list<person_info_t> person_list_t;
std::ostream& operator<<(std::ostream& out, const person_list_t & rPlist);

///target identifier structure
struct target_identifier_t{
  type_t type;///<type of the target
  target_id_t id;///identification number of the target

  ///constructor
  /**\param rType type of the target
   * \param rId identification number of the target
   */
  target_identifier_t(type_t rType, target_id_t rId);

  ///copy constructor
  /*\param rTargetIdentifier target identifier to copy
  */
  target_identifier_t(const target_identifier_t &rTargetIdentifier);

  ///operator == overloading
  /**equality check operator
   * \param rTargetIdentifier target_identifier_t to compare
   * \return result of the comparison
   */
  bool operator==(const target_identifier_t & rTargetIdentifier)const;

  ///operator != overloading
  /**non equality check operator
   * \param rTargetIdentifier target_identifier_t to compare
   * \return result of the comparison
   */
  bool operator!=(const target_identifier_t & rTargetIdentifier)const;

  ///overloading ostream operator << for target_identifier_t
  friend std::ostream& operator<<(std::ostream& out,
      const target_identifier_t &rTargetIdentifier);
};

///Target identifier list
typedef std::list<target_id_t> target_id_list_t;

///probability container
struct prob_element_t{
  float prob;///<probability for the target
  target_identifier_t target;///<target

  ///Constructor
  prob_element_t();

  ///Constructor
  /**\param rProbability probability
   * \param rTargetIdentifier target identifier
   */
  prob_element_t(const float &rProbability,
      const target_identifier_t &rTargetIdentifier);

  ///constructor
  /**\param rProbability probability
   * \param rType type
   * \param rId identification number
   */
  prob_element_t(const float &rProbability,const type_t &rType,
      const target_id_t &rId);

  ///copy constructor
  /**\param rProbElement prob_element_t to copy
  */
  prob_element_t(const prob_element_t &rProbElement);

  ///operator = overloading
  /** assignment operator overloading
   * \param rProbElement value to assign
   * \return *this
   */
  prob_element_t& operator=(const prob_element_t &rProbElement);

  ///ostream << operator overloading for prob_element_t
  friend std::ostream& operator<<(std::ostream& out,
      const prob_element_t &rProbElement);
};

///list of probabilities summing to one
typedef std::list<prob_element_t> distribution_t;
std::ostream& operator<<(std::ostream& out, const distribution_t &rDistrib);

///list of indepentant probabilities
typedef distribution_t probabilities_t;

///identified distribution
/**distribution with the id of the person to whom it applies*/
struct identified_distribution_t {

  target_id_t id;///<id of the person
  distribution_t distribution;///<distribution

  ///constructor
  /**\param rId identification number of the person
   * \rDistribution distribution of probabilities for the targets
   */
  identified_distribution_t(const target_id_t &rId,
    const distribution_t &rDistribution);

  ///ostream operator << overloading for identified_distribution_t
  friend std::ostream& operator<<(std::ostream& out,
    const identified_distribution_t &rIdDistrib);
};

///identified probabilities
/**probabilities with the id of the person to whom it applies*/
typedef identified_distribution_t identified_probabilities_t;

/// list of identified distributions
typedef std::list<identified_distribution_t>
    identified_distribution_list_t;
std::ostream& operator<<(std::ostream& out,
        const identified_distribution_list_t &rIdDistrib);

/// list of identified probabilities
typedef identified_distribution_list_t
    identified_probabilities_list_t;
#endif
