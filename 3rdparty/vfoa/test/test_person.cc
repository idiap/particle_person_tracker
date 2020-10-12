#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PersonTest
// #include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include "person.h"


struct PersonFixture{
  person_info_t tPersonInfo;
  person_list_t tPersonNewList;
  person_list_t tPersonNewDelList;
  person_list_t tPersonOldList;
  person_list_t tPersonInitList;
  object_list_t tObjectNewList;
  object_list_t tObjectNewDelList;
  object_list_t tObjectOldList;
  object_list_t tObjectInitList;
  Person *tPerson;
  boost::posix_time::ptime tTimestamp;

  //bool isfive(const object_info_t &o){return (o.id == 5);}
  PersonFixture(){
    tPersonInfo.id = 1;
    person_info_t self_person;
    object_info_t camera;
    person_info_t other_person;
    self_person.id = tPersonInfo.id;
    other_person.id = 2;
    camera.id = 21;
    self_person.loc.x = 1;
    self_person.loc.y = 1;
    self_person.loc.z = 1;
    other_person.loc.x = 4;
    other_person.loc.y = 2;
    other_person.loc.z = 3;
    camera.loc.x = 0;
    camera.loc.y = 0;
    camera.loc.z = 0;
    tObjectInitList.push_front(camera);
    tPersonInitList.push_front(self_person);
    tPersonInitList.push_front(other_person);
    tTimestamp = boost::posix_time::second_clock::local_time();
    for (int i = 1; i<12 ; i++){
      person_info_t person;
      object_info_t object;
      person.id = i;
      object.id = i+20;
      person.loc.x = i;
      person.loc.y = i+1;
      person.loc.z = i+2;
      object.loc.x = i;
      object.loc.y = i+2;
      object.loc.z = i+4;
      if (i != 5){
        tPersonNewList.push_front(person);
        tObjectNewList.push_front(object);
        if (i < 10){
          tPersonNewDelList.push_front(person);
          tObjectNewDelList.push_front(object);
        }
      }
      if (i < 11){
        tPersonOldList.push_front(person);
        tObjectOldList.push_front(object);
      }
    }
    assert(tPersonOldList.size()==10);
    assert(tObjectOldList.size()==10);
    assert(tPersonNewList.size()==10);
    assert(tObjectNewList.size()==10);
    assert(tPersonNewDelList.size()==8);
    assert(tObjectNewDelList.size()==8);
    tPerson = new Person(tPersonInfo.id,tObjectInitList,tPersonInitList,tTimestamp);
  }
  ~PersonFixture(){
    delete tPerson;
  }
  void CheckUpdated(object_list_t rObjectList, person_list_t rPersonList){
    vfoa_target_list_t target_list = tPerson->GetTargetList();
    // -1 for self, +1 for unfocussed so we just sum them
    BOOST_CHECK_EQUAL(rPersonList.size()+rObjectList.size(),target_list.size());
    vfoa_target_list_t::iterator ctlist_it;
    person_list_t::iterator plist_it;
    bool all_exist = true;
    for (ctlist_it = target_list.begin(); ctlist_it != target_list.end();
        ctlist_it++){
      bool this_exist = false;
      for (plist_it = rPersonList.begin(); plist_it != rPersonList.end();
          plist_it++){
        if ((*plist_it).id == (*ctlist_it).cId.id){
          this_exist = true;
        }
      }
      if (!this_exist && (*ctlist_it).cId.type == PERSON_INFO){
        all_exist = false;
      }
    }
    BOOST_CHECK(all_exist);
    object_list_t::iterator olist_it;
    all_exist = true;
    for (ctlist_it = target_list.begin(); ctlist_it != target_list.end();
        ctlist_it++){
      bool this_exist = false;
      for (olist_it = rObjectList.begin(); olist_it != rObjectList.end();
          olist_it++){
        if ((*olist_it).id == (*ctlist_it).cId.id){
          this_exist = true;
        }
      }
      if (!this_exist && (*ctlist_it).cId.type == OBJECT_INFO){
        all_exist = false;
      }
    }
    BOOST_CHECK(all_exist);
  }
  void CheckPlausibleVfoa(target_identifier_t rId, distribution_t rDistrib) {
    distribution_t::iterator distrib_it;
    prob_element_t max_prob;
    max_prob = rDistrib.front();
    for(distrib_it = rDistrib.begin(); distrib_it != rDistrib.end();
        distrib_it++){
      max_prob = (max_prob.prob < (*distrib_it).prob ? *distrib_it : max_prob);
    }
    BOOST_CHECK(max_prob.target == rId);
  }
  void CheckPlausibleProbability(target_identifier_t rId, probabilities_t rProb) {
    distribution_t::iterator distrib_it=rProb.begin();
    prob_element_t prob;
    bool exist = false;
    for(distrib_it; distrib_it != rProb.end(); distrib_it++){
      if ((*distrib_it).target == rId){
        prob = (*distrib_it);
        exist = true;
      }
    }
    BOOST_CHECK(prob.prob > 0.8 && exist);
  }
};

BOOST_FIXTURE_TEST_SUITE(person_test, PersonFixture)

BOOST_AUTO_TEST_CASE(test_constructor)
{
  CheckUpdated(tObjectInitList,tPersonInitList);
}

BOOST_AUTO_TEST_CASE(try_vfoa)
{
  BOOST_TEST_MESSAGE("try vfoa first case");
  tPersonInitList.back().head_pose = pose_t(0,0,0);
  for (int i = 0; i<500; i++){
    tTimestamp += boost::posix_time::millisec(40);
    tPerson->UpdatePerson(tObjectInitList,tPersonInitList,tTimestamp);
  }
  CheckPlausibleVfoa(target_identifier_t(OBJECT_INFO,tObjectInitList.back().id),
      tPerson->GetVfoaDistribution());
  CheckPlausibleVfoa(target_identifier_t(OBJECT_INFO,tObjectInitList.back().id),
      tPerson->GetVfoaProbabilities());
  CheckPlausibleProbability(target_identifier_t(OBJECT_INFO,tObjectInitList.back().id),
      tPerson->GetVfoaProbabilities());
  BOOST_TEST_MESSAGE("try vfoa second case");
  tPersonInitList.back().head_pose = pose_t(-2.7539959669346126,0,
      -0.26115741090302436);
  for (int i = 0; i<500; i++){
    tTimestamp += boost::posix_time::millisec(40);
    tPerson->UpdatePerson(tObjectInitList,tPersonInitList,tTimestamp);
  }
  CheckPlausibleVfoa(target_identifier_t(PERSON_INFO,
        tPersonInitList.front().id), tPerson->GetVfoaDistribution());
  CheckPlausibleVfoa(target_identifier_t(PERSON_INFO,
        tPersonInitList.front().id), tPerson->GetVfoaProbabilities());
  CheckPlausibleProbability(target_identifier_t(PERSON_INFO,
        tPersonInitList.front().id), tPerson->GetVfoaProbabilities());
  BOOST_TEST_MESSAGE("try vfoa third case");
  tPersonOldList.back().loc = location_t(1,1,1);
  tPersonOldList.back().head_pose = pose_t(2.7539959669346126,
      0.5796397403637044,0.509739678831507);
  tObjectOldList.back().loc = location_t(1,2,3);
  for (int i = 0; i<1000; i++){
    tTimestamp += boost::posix_time::millisec(40);
    tPerson->UpdatePerson(tObjectOldList,tPersonOldList,tTimestamp);
  }
  CheckPlausibleVfoa(target_identifier_t(OBJECT_INFO,tObjectOldList.back().id),
      tPerson->GetVfoaDistribution());
  CheckPlausibleVfoa(target_identifier_t(OBJECT_INFO,tObjectOldList.back().id),
      tPerson->GetVfoaProbabilities());
  CheckPlausibleProbability(target_identifier_t(OBJECT_INFO,tObjectOldList.back().id),
      tPerson->GetVfoaProbabilities());
  BOOST_TEST_MESSAGE("try vfoa fourth case");
  tPersonOldList.back().head_pose = pose_t(-0.1352519582446555,
      -0.2294371025070549,-0.1165383420016078);
  tObjectOldList.front().loc = location_t(-1,-2,-3);
  for (int i = 0; i<500; i++){
    tTimestamp += boost::posix_time::millisec(40);
    tPerson->UpdatePerson(tObjectOldList,tPersonOldList,tTimestamp);
  }
  CheckPlausibleVfoa(target_identifier_t(OBJECT_INFO,tObjectOldList.front().id),
      tPerson->GetVfoaDistribution());
  CheckPlausibleVfoa(target_identifier_t(OBJECT_INFO,tObjectOldList.front().id),
      tPerson->GetVfoaProbabilities());
  CheckPlausibleProbability(target_identifier_t(OBJECT_INFO,tObjectOldList.front().id),
      tPerson->GetVfoaProbabilities());
}

BOOST_AUTO_TEST_CASE(update_person_list)
{
  tTimestamp += boost::posix_time::millisec(40);
  tPerson->UpdatePerson(tObjectNewList,tPersonNewList,tTimestamp);
  CheckUpdated(tObjectNewList,tPersonNewList);
  BOOST_CHECK_EQUAL(tPerson->GetVfoaDistribution().size(),
      tObjectNewList.size()+tPersonNewList.size());
}

BOOST_AUTO_TEST_CASE(update_del_person_list)
{
  tTimestamp += boost::posix_time::millisec(40);
  tPerson->UpdatePerson(tObjectNewDelList,tPersonNewDelList,tTimestamp);
  CheckUpdated(tObjectNewDelList,tPersonNewDelList);
  BOOST_CHECK_EQUAL(tPerson->GetVfoaDistribution().size(),
      tObjectNewDelList.size()+tPersonNewDelList.size());
}

BOOST_AUTO_TEST_CASE(update_add_person_list)
{
  tTimestamp += boost::posix_time::millisec(40);
  tPerson->UpdatePerson(tObjectNewList,tPersonNewList,tTimestamp);
  CheckUpdated(tObjectNewList,tPersonNewList);
  BOOST_CHECK_EQUAL(tPerson->GetVfoaDistribution().size(),
      tObjectNewList.size()+tPersonNewList.size());
}

BOOST_AUTO_TEST_CASE(update_delete_all)
{
  object_list_t empty_object_list;
  person_list_t empty_person_list;
  empty_person_list.push_front(tPersonInfo);
  tTimestamp += boost::posix_time::millisec(40);
  tPerson->UpdatePerson(empty_object_list,empty_person_list,tTimestamp);
  CheckUpdated(empty_object_list,empty_person_list);
  BOOST_CHECK_EQUAL(tPerson->GetVfoaDistribution().size(),1);
}

BOOST_AUTO_TEST_SUITE_END()
