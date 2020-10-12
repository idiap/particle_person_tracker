/**
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include <boost/foreach.hpp>
#include <set>

#include "vfoa_manager.h"
#include "utils.h"

VfoaManager::VfoaManager() {
}

//VfoaManager::VfoaManager(const VfoaManager & rManager):
//  cVerbose(rManager.cVerbose), cOutputFile(rManager.cOutputFile),
//  mPersonList(rManager.mPersonList),mUpdateTimestamp(rManager.mUpdateTimestamp)
//{}

VfoaManager::~VfoaManager() {
    // delete all remaining person entries
    BOOST_FOREACH(const PersonMap::value_type& pentry, m_Persons) {
        delete pentry.second;
    }
    m_Persons.clear();
}

//VfoaManager& VfoaManager::operator=(const VfoaManager & rManager){
//  assert(cVerbose == rManager.cVerbose);
//  assert(cOutputFile == rManager.cOutputFile);
//  mPersonList = rManager.mPersonList;
//  mUpdateTimestamp = rManager.mUpdateTimestamp;
//  return *this;
//}

void VfoaManager::UpdateTargetList(object_list_t &rObjectInfoList,
    person_list_t &rPersonInfoList, const boost::posix_time::ptime &rTimestamp){

  mUpdateTimestamp = rTimestamp;

  mVfoaList.clear();
  mVfoaProbList.clear();
  m_PersonInfos.clear();
  m_ObjectInfos.clear();

  // set of IDs of all entries that have not been updated
  set<target_id_t> unupdated_entries;
  BOOST_FOREACH(const PersonMap::value_type& pentry, m_Persons) {
      unupdated_entries.insert(pentry.first);
  }

  // update existing entries, add new entries
  BOOST_FOREACH(const person_info_t& pinfo, rPersonInfoList) {
      m_PersonInfos[pinfo.id] = pinfo;
      PersonMap::iterator it_entry = m_Persons.find(pinfo.id);
      if (it_entry != m_Persons.end()) {
          // person is already managed, update the entry
          it_entry->second->UpdatePerson(rObjectInfoList, rPersonInfoList,
                  mUpdateTimestamp);
          identified_distribution_t new_distrib(pinfo.id,
                  distribution_t(it_entry->second->GetVfoaDistribution()));
          mVfoaList.push_front(new_distrib);
          identified_probabilities_t new_prob(pinfo.id,
                  distribution_t(it_entry->second->GetVfoaProbabilities()));
          mVfoaProbList.push_front(new_prob);
          unupdated_entries.erase(pinfo.id);
      } else {
          // person is not yet managed, add new entry
          Person * new_person = new Person(pinfo.id, rObjectInfoList,
                  rPersonInfoList, mUpdateTimestamp);
          m_Persons[pinfo.id] = new_person;
          identified_distribution_t new_distrib(pinfo.id,
                  distribution_t(new_person->GetVfoaDistribution()));
          mVfoaList.push_front(new_distrib);
          identified_probabilities_t new_prob(pinfo.id,
                  distribution_t(new_person->GetVfoaDistribution()));
          mVfoaProbList.push_front(new_prob);
      }
  }

  // erase entries for persons that do not exist any more
  BOOST_FOREACH(const target_id_t& pid, unupdated_entries) {
      PersonMap::iterator it_entry = m_Persons.find(pid);
      delete it_entry->second;
      m_Persons.erase(it_entry);
      m_PersonInfos.erase(pid);
  }

  // fill object infos into the map
  BOOST_FOREACH(const object_info_t& objinfo, rObjectInfoList) {
      m_ObjectInfos[objinfo.id] = objinfo;
  }

//  cout << " --=< VFOA manager iteration >=-- " << endl;
//  BOOST_FOREACH(const PersonMap::value_type& pentry, m_Persons) {
//      cout << *(pentry.second) << endl;
//  }
}

identified_distribution_list_t VfoaManager::GetVfoaList() const {
  return mVfoaList;
}

identified_probabilities_list_t VfoaManager::GetVfoaProbabilitiesList() const {
  return mVfoaProbList;
}

object_info_t VfoaManager::GetObjectInfo(const target_id_t& rId) const {
    ObjectInfoMap::const_iterator obj_info_it = m_ObjectInfos.find(rId);
    if (m_ObjectInfos.end() == obj_info_it) {
        return object_info_t();
    } else {
        return obj_info_it->second;
    }
}

person_info_t VfoaManager::GetPersonInfo(const target_id_t& rId) const {
    PersonInfoMap::const_iterator person_info_it = m_PersonInfos.find(rId);
    if (m_PersonInfos.end() == person_info_it) {
        return person_info_t();
    } else {
        return person_info_it->second;
    }
}

distribution_t VfoaManager::GetVfoaDistributionForId(const target_id_t &rId)
    const{
  identified_distribution_list_t::const_iterator distrib_it;
  for (distrib_it = mVfoaList.begin(); distrib_it != mVfoaList.end();
      distrib_it++){
    if ((*distrib_it).id == rId){
      return (*distrib_it).distribution;
    }
  }
  assert(false);
  return distribution_t();
}

probabilities_t VfoaManager::GetVfoaProbabilitiesForId(const target_id_t &rId)
    const{
  identified_probabilities_list_t::const_iterator prob_it;
  for (prob_it = mVfoaProbList.begin(); prob_it != mVfoaProbList.end();
      prob_it++){
    if ((*prob_it).id == rId){
      return (*prob_it).distribution;
    }
  }
  assert(false);
  return probabilities_t();
}

bool VfoaManager::TargetExists(const target_identifier_t &rTarget) const{
  identified_distribution_t distribution = mVfoaList.front();
  if (rTarget.type == PERSON_INFO && rTarget.id == distribution.id){
    return true;
  }
  distribution_t::const_iterator distrib_it;
  for (distrib_it = distribution.distribution.begin();
      distrib_it != distribution.distribution.end(); distrib_it++){
    if ((*distrib_it).target == rTarget){
      return true;
    }
  }
  return false;
}

std::ostream& operator<<(std::ostream& out, VfoaManager &rManager){
    out << "VFOA Manager dump [" << endl;
    out << "    Person list: [";
    BOOST_FOREACH(const VfoaManager::PersonMap::value_type& pentry,
            rManager.m_Persons) {
        out << pentry.second->GetPersonInfo() << ", ";
    }
    out << "]" << endl;
    out << "    VFOA distributions : " << rManager.mVfoaList << endl;
    out << "    VFOA probabilities : " << rManager.mVfoaProbList << endl;
    out << "]" << endl;
    return out;
} // operator<<
