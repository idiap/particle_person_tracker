#include "vfoa_manager.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <assert.h>
#include <fstream>

void CheckCorrectlyUpdated(const object_list_t &rObjectList,
    const person_list_t &rPersonList, VfoaManager &rManager){
  identified_distribution_list_t distribution_list = rManager.GetVfoaList();
  assert(distribution_list.size() == rPersonList.size());
  identified_distribution_list_t::iterator distribution_list_it;
  for (distribution_list_it=distribution_list.begin();
      distribution_list_it != distribution_list.end(); distribution_list_it++){
    assert(rPersonList.size()+rObjectList.size()==
        (*distribution_list_it).distribution.size());
  }
  distribution_list.clear();
}

int main(int argc, char** argv){
  std::ofstream *tOutputFile = new std::ofstream("vfoa-output.info");
  VfoaManager tManager;
  person_list_t tPersonOrigList;
  object_list_t tObjectOrigList;
  person_list_t tPersonEmptyList;
  object_list_t tObjectEmptyList;
  person_list_t tPersonDelList;
  object_list_t tObjectDelList;
  person_list_t tPersonAddList;
  object_list_t tObjectAddList;
  for(int i = 0; i < 11 ; i++){
    object_info_t pObject;
    pObject.id = i+10;
    person_info_t pPerson;
    pPerson.id = i;
    pPerson.loc.x = i;
    pPerson.loc.y = i+1;
    pPerson.loc.z = i+2;
    pObject.loc.x = i;
    pObject.loc.y = i+2;
    pObject.loc.z = i+4;
    tPersonAddList.push_front(pPerson);
    tObjectAddList.push_front(pObject);
    if (i < 10){
      tPersonOrigList.push_front(pPerson);
      tObjectOrigList.push_front(pObject);
      if (i != 5){
        tPersonDelList.push_front(pPerson);
        tObjectDelList.push_front(pObject);
      }
    }
  }
  *tOutputFile<<"load initial array"<<std::endl;
  tManager.UpdateTargetList(tObjectOrigList,tPersonOrigList,
      boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectOrigList,tPersonOrigList,tManager);
  *tOutputFile<<"update pose for person "<<tPersonOrigList.back().id<<std::endl;
  tPersonOrigList.back().head_pose.pan = 1;
  tPersonOrigList.back().head_pose.tilt = 2;
  tPersonOrigList.back().head_pose.roll = 3;
  tPersonOrigList.back().loc.x = 3;
  tPersonOrigList.back().loc.y = 2;
  tPersonOrigList.back().loc.z = 1;
  tManager.UpdateTargetList(tObjectOrigList,tPersonOrigList,
      boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectOrigList,tPersonOrigList,tManager);
  *tOutputFile<<tManager<<endl;
  *tOutputFile<<"add targets"<<std::endl;
  tManager.UpdateTargetList(tObjectAddList,
      tPersonAddList,boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectAddList,tPersonAddList,tManager);
  *tOutputFile<<tManager<<endl;
  *tOutputFile<<"delete some"<<std::endl;
  tManager.UpdateTargetList(tObjectDelList,
      tPersonDelList,boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectDelList,tPersonDelList,tManager);
  *tOutputFile<<tManager<<endl;
  *tOutputFile<<"delete all"<<std::endl;
  tManager.UpdateTargetList(tObjectEmptyList,
      tPersonEmptyList,boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectEmptyList,tPersonEmptyList,tManager);
  tOutputFile->close();
  delete tOutputFile;
}
