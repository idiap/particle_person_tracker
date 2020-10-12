#include "types.h"
#include <iterator>

/**
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

using namespace std;

pose_t::pose_t():pan(0),tilt(0),roll(0){}
pose_t::pose_t(float rPan, float rTilt, float rRoll):pan(rPan),tilt(rTilt),
  roll(rRoll){}
  pose_t::pose_t(const pose_t &rPose):pan(rPose.pan),
  tilt(rPose.tilt),
  roll(rPose.roll){}
  pose_t pose_t::operator +(pose_t rAddPose) {
    pose_t ret(*this);
    ret += rAddPose;
    return ret;
  }
void pose_t::operator +=(pose_t rAddPose){
  pan +=rAddPose.pan;
  tilt +=rAddPose.tilt;
  roll +=rAddPose.roll;
}
pose_t pose_t::operator *(float rMultiplier){
  pose_t ret(*this);
  ret.pan *= rMultiplier;
  ret.tilt *= rMultiplier;
  ret.roll *= rMultiplier;
  return ret;
}
pose_t pose_t::operator /(float rDivider){
  pose_t ret(*this);
  ret /= rDivider;
  return ret;
}
void pose_t::operator /= (float rDivider){
  pan /= rDivider;
  tilt /= rDivider;
  roll /= rDivider;
}
pose_t& pose_t::operator = (pose_t rEqualPose){
  pan = rEqualPose.pan;
  tilt = rEqualPose.tilt;
  roll = rEqualPose.roll;
  return *this;
}
std::ostream& operator<< (std::ostream &out, const pose_t &rPose) {
  out << "( pan : " << rPose.pan << ", tilt : " <<
    rPose.tilt << ", roll : " <<
    rPose.roll << ")";
  return out;
}

timestamped_pose_t::timestamped_pose_t(const pose_t &rPose,
    const boost::posix_time::ptime &rTimestamp):pose(rPose), timestamp(rTimestamp){}
timestamped_pose_t::timestamped_pose_t(const timestamped_pose_t
    &rTimestampedPose): pose(rTimestampedPose.pose),
    timestamp(rTimestampedPose.timestamp){}
timestamped_pose_t& timestamped_pose_t::operator=(const timestamped_pose_t &rTimestampedPose){
  pose = rTimestampedPose.pose;
  timestamp = rTimestampedPose.timestamp;
  return *this;
}

location_t::location_t():x(0),y(0),z(0){}
location_t::location_t(const float &rX, const float &rY, const float &rZ):x(rX),
  y(rY),z(rZ){}
  location_t::location_t(const location_t &rLocation): x(rLocation.x),
  y(rLocation.y),
  z(rLocation.z){}
location_t location_t::operator - (location_t rLocation){
    location_t ret(*this);
    ret.x -= rLocation.x;
    ret.y -= rLocation.y;
    ret.z -= rLocation.z;
    return ret;
}
location_t location_t::operator/(float rDivisor){
  location_t ret_loc(*this);
  ret_loc /= rDivisor;
  return ret_loc;
}

void location_t::operator /= (float rDivisor) {
    x /= rDivisor;
    y /= rDivisor;
    z /= rDivisor;
}

location_t& location_t::operator = (const location_t &rLocation){
  x = rLocation.x;
  y = rLocation.y;
  z = rLocation.z;
  return *this;
}
std::ostream& operator<< (std::ostream &out, const location_t &rLoc){
  out << "( x : " << rLoc.x << ", y : " <<
    rLoc.y << ", z : " <<
    rLoc.z << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const basis_t & rBasis) {
    out << "OX " << rBasis.axis[0] << endl;
    out << "OY " << rBasis.axis[1] << endl;
    out << "OZ " << rBasis.axis[2] << endl;
    return out;
}

object_info_t::object_info_t():id(1), name(), loc(){}
object_info_t::object_info_t(const object_info_t &rObjectInfo):
  id(rObjectInfo.id), name(rObjectInfo.name), loc(rObjectInfo.loc){}

std::ostream& operator<<(std::ostream &out, const object_info_t & rObjectInfo) {
    out<<"{ object info id "<<rObjectInfo.id<<", name : "<<rObjectInfo.name;
    out<<", location : "<<rObjectInfo.loc<<" }";
    return out;
}

std::ostream& operator<<(std::ostream& out, const object_list_t & rOlist) {
    out << "[object list :";
    std::ostream_iterator<object_info_t> out_it(out, ", ");
    std::copy(rOlist.begin(), rOlist.end(), out_it);
    out << "]";
    return out;
}

person_info_t::person_info_t():id(1), name(), loc(), head_pose(){}
person_info_t::person_info_t(const person_info_t &rPersonInfo):
  id(rPersonInfo.id), name(rPersonInfo.name), loc(rPersonInfo.loc),
  head_pose(rPersonInfo.head_pose){}
  person_info_t& person_info_t::operator = (person_info_t rPersonInfo){
    id=rPersonInfo.id;
    name=rPersonInfo.name;
    loc=rPersonInfo.loc;
    head_pose=rPersonInfo.head_pose;
    return *this;
  }
std::ostream& operator<<(std::ostream &out, const person_info_t & rPersonInfo) {
  out<<"{ person info id "<<rPersonInfo.id<<", name : "<<rPersonInfo.name;
  out<<", location : "<<rPersonInfo.loc<<", head pose :"<<rPersonInfo.head_pose;
  out<<" }";
  return out;
}
std::ostream& operator<<(std::ostream& out, const person_list_t & rPlist) {
  out << "[person list :";
  std::ostream_iterator<person_info_t> out_it(out, ", ");
  std::copy(rPlist.begin(), rPlist.end(), out_it);
  out << "]";
  return out;
}

target_identifier_t::target_identifier_t(type_t rType, target_id_t rId):
    type(rType), id(rId){}
target_identifier_t::target_identifier_t(
    const target_identifier_t &rTargetIdentifier):
  type(rTargetIdentifier.type), id(rTargetIdentifier.id){}
bool target_identifier_t::operator==(
    const target_identifier_t & rTargetIdentifier)const{
  return (rTargetIdentifier.id == id and rTargetIdentifier.type == type);
}
bool target_identifier_t::operator!=(
    const target_identifier_t & rTargetIdentifier)const{
  return (rTargetIdentifier.id != id or rTargetIdentifier.type != type);
}
std::ostream& operator<<(std::ostream &out,
    const target_identifier_t &rTargetIdentifier){
  out<<"( type = "<<rTargetIdentifier.type<<", id = "<<rTargetIdentifier.id;
  out<<" )";
  return out;
}

prob_element_t::prob_element_t():prob(0), target(ERROR,0){}
prob_element_t::prob_element_t(const float &rProbability,
    const target_identifier_t &rTargetIdentifier):
  prob(rProbability), target(rTargetIdentifier){}
  prob_element_t::prob_element_t(const float &rProbability,const type_t &rType,
      const target_id_t &rId):prob(rProbability), target(rType,rId){}
  prob_element_t::prob_element_t(const prob_element_t &rProbElement):
    prob(rProbElement.prob), target(rProbElement.target){}
    prob_element_t& prob_element_t::operator=(const prob_element_t &rProbElement){
      target = rProbElement.target;
      prob = rProbElement.prob;
      return *this;
    }
std::ostream& operator<<(std::ostream& out, const prob_element_t & rProbElement){
  out<<"( target : "<<rProbElement.target<<",  probability : ";
  out<<rProbElement.prob<<" )";
  return out;
}
std::ostream& operator<<(std::ostream& out, const distribution_t &rDistrib){
  out<<"{distribution:";
  std::ostream_iterator<prob_element_t> out_it(out, ", ");
  std::copy(rDistrib.begin(), rDistrib.end(), out_it);
  out<<"}";
  return out;
}

identified_distribution_t::identified_distribution_t(const target_id_t &rId,
    const distribution_t &rDistribution): id(rId), distribution(rDistribution) {}

std::ostream& operator<<(std::ostream &out,
    const identified_distribution_t & rIdDistrib) {
    out << "distribution for target : "<< rIdDistrib.id << std::endl;
    out << rIdDistrib.distribution << std::endl;
    return out;
}
std::ostream& operator<<(std::ostream& out,
        const identified_distribution_list_t &rIdDistrib){
    out << "[identified distribution list:" << std::endl;
    std::ostream_iterator<identified_distribution_t> out_it(out, ", ");
    std::copy(rIdDistrib.begin(), rIdDistrib.end(), out_it);
    out<<"]" << std::endl;
    return out;
}
