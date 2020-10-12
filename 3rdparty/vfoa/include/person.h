/**
 * @file include/person.h
 * @date 07 March 2013
 * @author Salim Kayal <salim.kayal@idiap.ch>
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Class to describe a person within the visual focus of attention
 * manager
 *
 * Copyright (C) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef PERSON_H
#define PERSON_H

// SYSTEM INCLUDES
#include <list>
#include <boost/circular_buffer.hpp>

// LOCAL INCLUDES
#include "vfoa_target.h"

static const int HP_BUFFER_SIZE = 1000;
static const boost::posix_time::time_duration
    HP_BUFFER_TIME_DURATION_FOR_REF_POSE_ESTIMATION =
    boost::posix_time::seconds(30);
static const boost::posix_time::time_duration HP_BUFFER_TIME_DURATION_FOR_PREVIOUS_HP_ESTIMATION =
    boost::posix_time::seconds(5);
static const boost::posix_time::time_duration HP_BUFFER_GAP_FOR_PREVIOUS_HP_ESTIMATION =
    boost::posix_time::seconds(5);

template<typename T> bool delete_type(const VfoaTarget& rTarget){
  if (rTarget.cId.type == utils::Type<T>()){
    return true;
  }
  return false;
}

class Person {

  public:

    // LIFECYCLE

    /// Construct person VFOA manager entry using person's ID,
    /// list of currently observed objects and persons. Note that the list of
    /// observed persons contains the person for which this entry is being
    /// constructed.
    /// @param rId Person's ID
    /// @param rObjectList List of currently observed objects
    /// @param rObjectList List of currently observed persons
    /// @param rTimestamp Time at which the current entry is being created
    Person(target_id_t rId,object_list_t rObjectList,person_list_t rPersonList,
        boost::posix_time::ptime rTimestamp);

//    /// Constructs a copy of person's entry
//    /// @param rPerson Person's entry
//    Person(const Person& rPerson);

    /// Destroys person's entry
    ~Person();

    // OPERATIONS

//    /// Assignment operator to assign other person's entry data to this
//    /// @param rPerson Other person's entry
//    /// @return Modified this person's entry
//    Person& operator=(const Person & rPerson);

    ///update target_list and head pose
    void UpdatePerson(object_list_t rObjectList,person_list_t rPersonList,
        boost::posix_time::ptime rTimestamp);

    /// return computed vfoa
    distribution_t GetVfoaDistribution();

    probabilities_t GetVfoaProbabilities();

    person_info_t GetPersonInfo();

    vfoa_target_list_t GetTargetList();


  private:
    Person(const Person& rPerson);
    Person& operator=(const Person & rPerson);

    // basic information on the person: ID, name, current location and head pose
    person_info_t m_CurrentPersonInfo;
    /// list of targets
    vfoa_target_list_t mTargets;
    /// transition matrix
    float *mTransitionMatrix;
    static const float cTwoTargetTransitionMatrix[];
    /// distribution for vfoa
    distribution_t mDistrib;
    /// distribution for vfoa
    probabilities_t mProb;
    ///circular buffer of head pose (necessary for R and u_h computation)
    boost::circular_buffer<timestamped_pose_t> mHeadPose;
    // reference head pose of the person
    pose_t m_ReferenceHeadPose;
    // previous head pose of the person
    pose_t m_PreviousHeadPose;

    /// calculate mean from head pose circular buffer on n samples
    pose_t CalcHeadPoseMeanOnNSamples(boost::posix_time::time_duration rNSec,
        boost::posix_time::time_duration rGapSec =
        boost::posix_time::seconds(0));
    bool UpdateCurrentPersonInfo(person_list_t &rPList);

    /// complete and update the target list
    template<typename T>
      void UpdateList(list<T> rNewList, const bool &rUpdateUnfocussed = false) {
        //static_assert((sizeof(T) != sizeof(person_info_t)) ||
        //    (sizeof(T) != sizeof(object_info_t)),
        //    "You are missing a DECL_TYPE_NAME");
        vfoa_target_list_t::iterator it_old_list;
        if(rNewList.size()==0 ||
            (rNewList.size()==1 &&
             rNewList.front().id == m_CurrentPersonInfo.id &&
             utils::Type<T>() == PERSON_INFO)) {
          mTargets.remove_if(delete_type<T>);
          return;
        }
        typename list<T>::iterator it_new_list;
        for (it_old_list = mTargets.begin(); it_old_list != mTargets.end();
            it_old_list++){
          bool keep = false;
          bool unfocussed_updated=false;
          if ((*it_old_list).cId.type == UNFOCUSSED_VALUE && rUpdateUnfocussed){
            assert(!unfocussed_updated);
            unfocussed_updated = true;
            (*it_old_list).UpdateTarget(location_t());
          }
          for (it_new_list = rNewList.begin();it_new_list != rNewList.end();
              it_new_list++){
            if ((*it_old_list).cId.id == (*it_new_list).id &&
                (*it_old_list).cId.type == utils::Type<T>()){
              keep = true;
            }
          }
          if (!keep &&
              utils::Type<T>() == (*it_old_list).cId.type &&
              (*it_old_list).cId.type != UNFOCUSSED_VALUE){
            it_old_list = mTargets.erase(it_old_list);
          }
        }
        for ( it_new_list = rNewList.begin(); it_new_list != rNewList.end();
            it_new_list++){
          bool updated = false;
          for (it_old_list = mTargets.begin(); it_old_list != mTargets.end();
              it_old_list++){
            if ((*it_old_list).cId.id == (*it_new_list).id &&
                (*it_old_list).cId.type == utils::Type<T>()){
              (*it_old_list).UpdateTarget((*it_new_list).loc);
              updated = true;
            }
          }
          if (!updated && (*it_new_list).id != m_CurrentPersonInfo.id){
            mTargets.push_front(VfoaTarget(*it_new_list, *this));
          }
        }
      }

    /// compute transition matrix
    void ComputeTransitionMatrix();

    // FRIENDS

    // person's VFOA targets should be able to access person's data
    friend class VfoaTarget;
    // output operator
    friend std::ostream& operator<<(std::ostream& out, const Person &rPerson);

};

std::ostream& operator<<(std::ostream& out, const Person &rPerson);

#endif // PERSON_H
