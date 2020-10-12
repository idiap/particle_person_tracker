#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE VfoaManagerTest
// #include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>
#include "vfoa_manager.h"


struct VfoaManagerFixture{
  VfoaManager tManager;
  person_list_t tPersonOrigList;
  object_list_t tObjectOrigList;
  person_list_t tPersonEmptyList;
  object_list_t tObjectEmptyList;
  person_list_t tPersonDelList;
  object_list_t tObjectDelList;
  person_list_t tPersonAddList;
  object_list_t tObjectAddList;
  VfoaManagerFixture(){
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
  }
  ~VfoaManagerFixture(){
    tPersonAddList.clear();
    tObjectAddList.clear();
  }
  void CheckCorrectlyUpdated(object_list_t rObjectList,
      person_list_t rPersonList){
    identified_distribution_list_t distribution_list = tManager.GetVfoaList();
    BOOST_CHECK_EQUAL(distribution_list.size(), rPersonList.size());
    identified_distribution_list_t::iterator distribution_list_it;
    for (distribution_list_it=distribution_list.begin();
        distribution_list_it != distribution_list.end();
        distribution_list_it++){
      BOOST_CHECK_EQUAL(rPersonList.size()+rObjectList.size(),
          (*distribution_list_it).distribution.size());
    }
    distribution_list.clear();
  }
};

BOOST_FIXTURE_TEST_SUITE(manager_test, VfoaManagerFixture)

BOOST_AUTO_TEST_CASE(create_origin){
  tManager.UpdateTargetList(tObjectOrigList,
      tPersonOrigList,boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectOrigList,tPersonOrigList);
}

BOOST_AUTO_TEST_CASE(add_targets){
  tManager.UpdateTargetList(tObjectAddList,tPersonAddList,
      boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectAddList,tPersonAddList);
}

BOOST_AUTO_TEST_CASE(delete_some){
  tManager.UpdateTargetList(tObjectDelList,tPersonDelList,
      boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectDelList,tPersonDelList);
}

BOOST_AUTO_TEST_CASE(delete_all){
  tManager.UpdateTargetList(tObjectEmptyList,tPersonEmptyList,
      boost::posix_time::second_clock::local_time());
  CheckCorrectlyUpdated(tObjectEmptyList,tPersonEmptyList);
}

BOOST_AUTO_TEST_SUITE_END()
