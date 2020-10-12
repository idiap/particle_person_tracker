#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PersonTest
#include <boost/test/unit_test.hpp>
#include "vfoa_target.h"
#include "utils.h"

struct VfoaTargetFixture{
  vfoa_target_list_t tTargetList;
  target_identifier_t tId;
  location_t tTargetLocation;
  location_t *ptRefLocation;
  pose_t *ptHeadPose;
  pose_t *ptReferencePosition;
  pose_t *ptPreviousHeadPose;
  VfoaTargetFixture():tId(OBJECT_INFO,1),
  tTargetLocation(-1,-1,-1),
  ptRefLocation(new location_t(1,1,1)),
  ptHeadPose(new pose_t(utils::PI/16,utils::PI/8,0)),
  ptReferencePosition(new pose_t(0,0,0)),
  ptPreviousHeadPose(new pose_t(0,0,0)){
    BOOST_TEST_MESSAGE("fixture setup");
    tTargetList.push_front(VfoaTarget(tId.id,tId.type,tTargetLocation,
    ptRefLocation,ptHeadPose,ptReferencePosition,ptPreviousHeadPose));

  }
  ~VfoaTargetFixture(){
    BOOST_TEST_MESSAGE("fixture teardown");
    tTargetList.clear();
    delete ptRefLocation;
    delete ptHeadPose;
    delete ptReferencePosition;
    delete ptPreviousHeadPose;
  }
};

BOOST_FIXTURE_TEST_SUITE(vfoa_target_test, VfoaTargetFixture);

BOOST_AUTO_TEST_CASE(constructor){
  BOOST_CHECK_EQUAL(tTargetList.size(),1);
  BOOST_CHECK_EQUAL(tTargetList.front().cId.id, tId.id);
  BOOST_CHECK_EQUAL(tTargetList.front().cId.type, tId.type);
  BOOST_CHECK_CLOSE(tTargetList.front().GetMean().pan,0,0.001);
  BOOST_CHECK_CLOSE(tTargetList.front().GetMean().tilt,0,0.001);
  BOOST_CHECK_CLOSE(tTargetList.front().GetMean().roll,0,0.001);
  BOOST_CHECK_CLOSE(tTargetList.front().GetGase().pan,0,0.001);
  BOOST_CHECK_CLOSE(tTargetList.front().GetGase().tilt,0,0.001);
}

BOOST_AUTO_TEST_CASE(check_gase)
{
  tTargetList.front().UpdateTarget(location_t(1,2,3));

  BOOST_CHECK_CLOSE(tTargetList.front().GetGase().pan,1.65239758,0.001);
  BOOST_CHECK_CLOSE(tTargetList.front().GetGase().tilt,0.231855896145,0.001);

  ptReferencePosition->pan = utils::PI/8;
  ptReferencePosition->tilt = utils::PI/8;
  ptPreviousHeadPose->pan = utils::PI/16;
  ptPreviousHeadPose->tilt = utils::PI/8;

  tTargetList.front().UpdateTarget(location_t(1,2,3));
  BOOST_CHECK_CLOSE(tTargetList.front().GetGase().pan,1.809477213,0.001);
  BOOST_CHECK_CLOSE(tTargetList.front().GetGase().tilt,0.467475345165,0.001);
}

BOOST_AUTO_TEST_SUITE_END();
