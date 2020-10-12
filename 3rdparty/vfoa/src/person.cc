/**
 * @file src/person.cc
 * @date 07 March 2013
 * @author Salim Kayal <salim.kayal@idiap.ch>
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Class to describe a person within the visual focus of attention
 * manager
 *
 * Copyright (C) 2011-2012 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <math.h>
#include <assert.h>
#include <boost/foreach.hpp>                            // boost foreach loop

// LOCAL INCLUDES
#include "person.h"                                     // declaration of this

/////////////////////////////// CONSTANTS ////////////////////////////////////

// probability for an HMM to stay in the same state
static const float P_I_I = 0.6;

using namespace std;

/////////////////////////////// PUBLIC ///////////////////////////////////////

/*******constructor and destructor*******/
Person::Person(target_id_t rId,object_list_t rObjectList,
    person_list_t rPersonList, boost::posix_time::ptime rTimestamp):
    mTransitionMatrix(NULL),mHeadPose(HP_BUFFER_SIZE) {

    // fill person targets and information for this person
    bool assert_this_person_exists = false;
    BOOST_FOREACH(const person_info_t& pinfo, rPersonList) {
        if (pinfo.id == rId) {
            m_CurrentPersonInfo = pinfo;
            assert_this_person_exists = true;
        } else {
            VfoaTarget new_target(pinfo, *this);
            mTargets.push_front(new_target);
        }
    }
    assert(assert_this_person_exists);

    // fill object targets
    BOOST_FOREACH(const object_info_t& objinfo, rObjectList) {
        VfoaTarget new_target(objinfo, *this);
        mTargets.push_front(new_target);
    }

    // add UNFOCUSED
    VfoaTarget new_target(*this);
    mTargets.push_front(new_target);

    // initialize head pose buffer for reference estimation
    for (int i = HP_BUFFER_SIZE; i>0; i--){
      boost::posix_time::time_duration delay = boost::posix_time::millisec(40*i);
      timestamped_pose_t timestamped_pose(pose_t(0,0,0),
          rTimestamp - delay);
      //std::cout<<delay<<std::endl;
      mHeadPose.push_front(timestamped_pose);
    }
    mHeadPose.push_front(timestamped_pose_t(m_CurrentPersonInfo.head_pose,rTimestamp));

    // initialize transition matrix
    float init_prev_prob;
    init_prev_prob = 1./mTargets.size();
    vfoa_target_list_t::iterator target_it;
    for (target_it = mTargets.begin(); target_it != mTargets.end();
        target_it++) {
      mDistrib.push_front(prob_element_t(init_prev_prob,(*target_it).cId));
      if ((*target_it).cId.type != UNFOCUSSED_VALUE){
        distribution_t two_elements_distrib;
        two_elements_distrib.push_front(prob_element_t(0.5, new_target.cId));
        two_elements_distrib.push_front(prob_element_t(0.5, (*target_it).cId));
        vfoa_target_list_t two_elements_targets;
        two_elements_targets.push_front(new_target);
        two_elements_targets.push_front(*target_it);
        distribution_t target_transition_probability =
            utils::compute_transition_probabilities(two_elements_targets,
            two_elements_distrib,cTwoTargetTransitionMatrix);
        mProb.push_front(utils::compute_posterior(two_elements_targets,
            target_transition_probability).front());
      }
    }
    assert(mTransitionMatrix == NULL);
    ComputeTransitionMatrix();
    assert(mTransitionMatrix != NULL);
    distribution_t transition_probabilities =
        utils::compute_transition_probabilities(mTargets,mDistrib,
        mTransitionMatrix);
    mDistrib = utils::compute_posterior(mTargets, transition_probabilities);
  }

//Person::Person(const Person & rPerson):
//        m_CurrentPersonInfo(rPerson.m_CurrentPersonInfo),
//        mTargets(rPerson.mTargets),
//        mTransitionMatrix(NULL),
//        mDistrib(rPerson.mDistrib),
//        mHeadPose(rPerson.mHeadPose),
//        m_ReferenceHeadPose(rPerson.m_ReferenceHeadPose),
//        m_PreviousHeadPose(rPerson.m_PreviousHeadPose) {
//    vfoa_target_list_t::iterator target_it;
//    for (target_it = mTargets.begin(); target_it != mTargets.end();
//         target_it++){
//      (*target_it).UpdateReferencePerson(&(m_CurrentPersonInfo.loc),
//                                         &(m_CurrentPersonInfo.head_pose),
//                                         &(m_ReferenceHeadPose),
//                                         &(m_PreviousHeadPose));
//    }
//    assert(mTransitionMatrix == NULL);
//    ComputeTransitionMatrix();
//    assert(mTransitionMatrix != NULL);
//  }

Person::~Person(){
  if (mTransitionMatrix != NULL){
    delete[] mTransitionMatrix;
    mTransitionMatrix = NULL;
  }
}

Person& Person::operator=(const Person & rPerson) {
  m_CurrentPersonInfo = rPerson.m_CurrentPersonInfo;
  mHeadPose=rPerson.mHeadPose;
  m_ReferenceHeadPose = rPerson.m_ReferenceHeadPose;
  m_PreviousHeadPose = rPerson.m_PreviousHeadPose;
  ComputeTransitionMatrix();
  mDistrib = rPerson.mDistrib;
  mProb = rPerson.mProb;
  return *this;
}

/*****************public*****************/
void Person::UpdatePerson(object_list_t rObjectList,person_list_t rPersonList,
    boost::posix_time::ptime rTimestamp){
  bool hasupdated = UpdateCurrentPersonInfo(rPersonList);
  assert(hasupdated);
  mHeadPose.push_front(timestamped_pose_t(m_CurrentPersonInfo.head_pose,
        rTimestamp));
  m_ReferenceHeadPose = CalcHeadPoseMeanOnNSamples(
      HP_BUFFER_TIME_DURATION_FOR_REF_POSE_ESTIMATION);
  m_PreviousHeadPose = CalcHeadPoseMeanOnNSamples(
      HP_BUFFER_TIME_DURATION_FOR_PREVIOUS_HP_ESTIMATION,
      HP_BUFFER_GAP_FOR_PREVIOUS_HP_ESTIMATION);
  UpdateList<object_info_t>(rObjectList);
  UpdateList<person_info_t>(rPersonList, true);

  vfoa_target_list_t::const_iterator target_it;
  VfoaTarget unfocused(*this);
  probabilities_t old_prob(mProb);
  mProb.clear();
  for (target_it = mTargets.begin(); target_it != mTargets.end(); target_it++){
    if ((*target_it).cId.type != UNFOCUSSED_VALUE){
      distribution_t two_elements_distrib;
      probabilities_t::iterator prob_it;
      target_identifier_t id(UNFOCUSSED_VALUE, unfocused.cId.id);
      bool new_el = true;
      prob_element_t prob;
      for (prob_it = old_prob.begin(); prob_it != old_prob.end(); prob_it++){
        if ((*prob_it).target == (*target_it).cId){
          prob = (*prob_it);
          new_el = false;
        }
      }
      if (new_el){
        //std::cout<<"here"<<std::endl;
        two_elements_distrib.push_front(prob_element_t(0.5, unfocused.cId));
        two_elements_distrib.push_front(prob_element_t(0.5, (*target_it).cId));
      }
      else{
        //std::cout<<"there"<<std::endl;
        two_elements_distrib.push_front(prob_element_t(1 - prob.prob, unfocused.cId));
        two_elements_distrib.push_front(prob_element_t(prob.prob, (*target_it).cId));
      }
      vfoa_target_list_t two_elements_targets;
      two_elements_targets.push_front(unfocused);
      two_elements_targets.push_front(*target_it);
      distribution_t target_transition_probability =
        utils::compute_transition_probabilities(two_elements_targets,
            two_elements_distrib,cTwoTargetTransitionMatrix);
      mProb.push_front(utils::compute_posterior(two_elements_targets,
            target_transition_probability).back());
    }
  }
  ComputeTransitionMatrix();
  distribution_t transition_probabilities =
    utils::compute_transition_probabilities(mTargets,mDistrib,
        mTransitionMatrix);
  mDistrib = utils::compute_posterior(mTargets,transition_probabilities);
}

/****************getters*****************/
distribution_t Person::GetVfoaDistribution(){return mDistrib;}

probabilities_t Person::GetVfoaProbabilities(){return mProb;}

person_info_t Person::GetPersonInfo(){return m_CurrentPersonInfo;}

vfoa_target_list_t Person::GetTargetList(){return mTargets;}

/****************private*****************/
const float Person::cTwoTargetTransitionMatrix[] = {P_I_I, 1-P_I_I,1-P_I_I,P_I_I};

bool Person::UpdateCurrentPersonInfo(person_list_t &rPersonList){
    BOOST_FOREACH(const person_info_t& pinfo, rPersonList) {
        if (pinfo.id == m_CurrentPersonInfo.id) {
            m_CurrentPersonInfo = pinfo;
            return true;
        }
    }
    return false;
}

pose_t Person::CalcHeadPoseMeanOnNSamples(
    boost::posix_time::time_duration rNSec,
    boost::posix_time::time_duration rGapSec){
  pose_t mean;
  boost::posix_time::time_duration full_length = mHeadPose[0].timestamp -
    mHeadPose[HP_BUFFER_SIZE-1].timestamp;
  rGapSec = (rGapSec < full_length ? rGapSec:full_length);
  rNSec = (rNSec + rGapSec < full_length ? rNSec:full_length - rGapSec);
  int i = 0;
  while (rGapSec > mHeadPose[0].timestamp - mHeadPose[i].timestamp){
    //std::cout<<mHeadPose[0].timestamp - mHeadPose[i].timestamp<<std::endl;
    i++;
  }
  float n(0);
  //std::cout<<"calculate"<<std::endl;
  while (rGapSec + rNSec > mHeadPose[0].timestamp - mHeadPose[i].timestamp){
    //std::cout<<"calc"<<mHeadPose[i].timestamp<<std::endl;
    boost::posix_time::time_duration delta = mHeadPose[i].timestamp - mHeadPose[i+1].timestamp;
    n+= (float)delta.total_milliseconds();
    mean += mHeadPose[i].pose * (float)delta.total_milliseconds();
    i++;
  }
  //std::cout<<rGapSec<<' '<<rNSec<<' '<<n<<endl;
  mean /= n;
  return mean;
}

void Person::ComputeTransitionMatrix(){
  if (mTransitionMatrix != NULL){
    delete[] mTransitionMatrix;
    mTransitionMatrix = NULL;
  }
  int targets_size = mTargets.size();
  int prob_size = mDistrib.size();
  //std::cout<<targets_size<<"x"<<mDistrib.size();
  mTransitionMatrix = new float[targets_size*prob_size];
  float * matrix_begin_address = mTransitionMatrix;
  float p_i_j;
  p_i_j = ( 1 - P_I_I ) / targets_size;
  for (vfoa_target_list_t::iterator target_it = mTargets.begin();
       target_it != mTargets.end(); target_it++){
    bool i_i_exists = false;
    float * line_begin_address = mTransitionMatrix;
    for(distribution_t::iterator prob_it = mDistrib.begin();prob_it != mDistrib.end();
        prob_it ++){
      if ((*prob_it ).target== (*target_it).cId){
        *mTransitionMatrix = P_I_I;
        i_i_exists = true;
      }
      else{
        *mTransitionMatrix = p_i_j;
      }
      mTransitionMatrix++;
    }
    if (! i_i_exists){
      mTransitionMatrix = line_begin_address;
      float p_all;
      p_all = 1. / targets_size;
      for (int j = 0; j<prob_size; j++){
        *mTransitionMatrix = p_all;
        mTransitionMatrix++;
      }
    }
  }
  mTransitionMatrix = matrix_begin_address;
}


ostream& operator<<(ostream& out, const Person &rPerson) {
    out << rPerson.m_CurrentPersonInfo << endl;
    out << rPerson.mTargets << endl;
    return out;
}
