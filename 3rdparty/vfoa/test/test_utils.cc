#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE UtilsTest
// #include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>
#include <utils.h>


struct UtilsFixture{
  location_t l1;
  location_t l2;
  location_t l3;
  location_t l4;
  location_t P;
  location_t NAO;
  location_t P1;
  location_t P2;
  location_t P3;
  location_t P4;

  basis_t look_nao;
  basis_t look_P1;
  basis_t look_P2;
  basis_t look_P3;
  basis_t look_P4;

  basis_t rotated_P1;
  basis_t rotated_P2;
  basis_t rotated_P3;
  basis_t rotated_P4;

  pose_t angles_P1;
  pose_t angles_P2;
  pose_t angles_P3;
  pose_t angles_P4;

  UtilsFixture():l1(1,1,1), l2(2,2,2), l3(1,2,3), l4(4,5,6),
  P(1,1,1), NAO(0,0,0), P1(-1,-1,-1), P2(2,2,2), P3(-2,-2,-2), P4(1,2,3),
  look_nao(utils::make_basis_from_positions(NAO,P)),
  look_P1(utils::make_basis_from_positions(P1,P)),
  look_P2(utils::make_basis_from_positions(P2,P)),
  look_P3(utils::make_basis_from_positions(P3,P)),
  look_P4(utils::make_basis_from_positions(P4,P)),

  rotated_P1(utils::calculate_rotation(look_nao, look_P1)),
  rotated_P2(utils::calculate_rotation(look_nao, look_P2)),
  rotated_P3(utils::calculate_rotation(look_nao, look_P3)),
  rotated_P4(utils::calculate_rotation(look_nao, look_P4)),

  angles_P1(utils::base_to_euler_YZX(rotated_P1)),
  angles_P2(utils::base_to_euler_YZX(rotated_P2)),
  angles_P3(utils::base_to_euler_YZX(rotated_P3)),
  angles_P4(utils::base_to_euler_YZX(rotated_P4)){
  }

  ~UtilsFixture(){}
};

BOOST_FIXTURE_TEST_SUITE(utils_test, UtilsFixture)

BOOST_AUTO_TEST_CASE(dot_product)
{
  BOOST_CHECK_CLOSE(utils::dot(l1,l2),6,0.001);
  BOOST_CHECK_CLOSE(utils::dot(l3,l4),32,0.001);
  BOOST_CHECK_CLOSE(utils::dot(l2,l4),30,0.001);
}

BOOST_AUTO_TEST_CASE(cross_product)
{
  location_t cross_l12 = utils::cross(l1,l2);
  location_t cross_l34 = utils::cross(l3,l4);
  location_t cross_l14 = utils::cross(l1,l4);
  BOOST_CHECK_CLOSE(cross_l12.x,0,0.001);
  BOOST_CHECK_CLOSE(cross_l12.y,0,0.001);
  BOOST_CHECK_CLOSE(cross_l12.z,0,0.001);
  BOOST_CHECK_CLOSE(cross_l34.x,-3,0.001);
  BOOST_CHECK_CLOSE(cross_l34.y,6,0.001);
  BOOST_CHECK_CLOSE(cross_l34.z,-3,0.001);
  BOOST_CHECK_CLOSE(cross_l14.x,1,0.001);
  BOOST_CHECK_CLOSE(cross_l14.y,-2,0.001);
  BOOST_CHECK_CLOSE(cross_l14.z,1,0.001);
}

BOOST_AUTO_TEST_CASE(make_basis)
{
  BOOST_CHECK_CLOSE(look_nao.axis[0].x,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[1].x,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[2].x,-0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[0].y,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[1].y,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[2].y,0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[0].z,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[1].z,-0.81649658,0.001);
  BOOST_CHECK_CLOSE(look_nao.axis[2].z,0,0.001);

  BOOST_CHECK_CLOSE(look_P1.axis[0].x,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[1].x,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[2].x,-0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[0].y,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[1].y,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[2].y,0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[0].z,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[1].z,-0.81649658,0.001);
  BOOST_CHECK_CLOSE(look_P1.axis[2].z,0,0.001);

  BOOST_CHECK_CLOSE(look_P2.axis[0].x,0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[1].x,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[2].x,0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[0].y,0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[1].y,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[2].y,-0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[0].z,0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[1].z,-0.81649658,0.001);
  BOOST_CHECK_CLOSE(look_P2.axis[2].z,0,0.001);

  BOOST_CHECK_CLOSE(look_P3.axis[0].x,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[1].x,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[2].x,-0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[0].y,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[1].y,0.40824829,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[2].y,0.70710678,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[0].z,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[1].z,-0.81649658,0.001);
  BOOST_CHECK_CLOSE(look_P3.axis[2].z,0,0.001);

  BOOST_CHECK_CLOSE(look_P4.axis[0].x,0,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[1].x,0,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[2].x,1,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[0].y,0.4472136,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[1].y,0.89442719,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[2].y,0,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[0].z,0.89442719,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[1].z,-0.4472136,0.001);
  BOOST_CHECK_CLOSE(look_P4.axis[2].z,0,0.001);
}

BOOST_AUTO_TEST_CASE(calculate_rotation)
{
  BOOST_CHECK_CLOSE(rotated_P1.axis[0].x,1,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[1].x,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[2].x,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[0].y,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[1].y,1,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[2].y,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[0].z,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[1].z,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P1.axis[2].z,1,0.001);

  BOOST_CHECK_CLOSE(rotated_P2.axis[0].x,-1,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[1].x,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[2].x,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[0].y,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[1].y,1,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[2].y,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[0].z,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[1].z,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P2.axis[2].z,-1,0.001);

  BOOST_CHECK_CLOSE(rotated_P3.axis[0].x,1,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[1].x,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[2].x,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[0].y,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[1].y,1,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[2].y,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[0].z,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[1].z,0,0.001);
  BOOST_CHECK_CLOSE(rotated_P3.axis[2].z,1,0.001);

  BOOST_CHECK_CLOSE(rotated_P4.axis[0].x,-0.77459667,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[1].x,-0.25819889,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[2].x,-0.57735027,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[0].y,-0.54772256,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[1].y,0.73029674,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[2].y,0.40824829,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[0].z,0.31622777,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[1].z,.63245553,0.001);
  BOOST_CHECK_CLOSE(rotated_P4.axis[2].z,-0.70710678,0.001);
}

BOOST_AUTO_TEST_CASE(base_to_euler)
{
  BOOST_CHECK_CLOSE(angles_P1.pan,0,0.001);
  BOOST_CHECK_CLOSE(angles_P1.tilt,0,0.001);
  BOOST_CHECK_CLOSE(angles_P1.roll,0,0.001);
  BOOST_CHECK_CLOSE(angles_P2.pan,3.141592653589793,0.001);
  BOOST_CHECK_CLOSE(angles_P2.tilt,0,0.001);
  BOOST_CHECK_CLOSE(angles_P2.roll,0,0.001);
  BOOST_CHECK_CLOSE(angles_P3.pan,0,0.001);
  BOOST_CHECK_CLOSE(angles_P3.tilt,0,0.001);
  BOOST_CHECK_CLOSE(angles_P3.roll,0,0.001);
  BOOST_CHECK_CLOSE(angles_P4.pan,2.7539959669346126,0.001);
  BOOST_CHECK_CLOSE(angles_P4.tilt,0.5796397403637044,0.001);
  BOOST_CHECK_CLOSE(angles_P4.roll,0.509739678831507,0.001);
}

BOOST_AUTO_TEST_CASE(check_type)
{
//  BOOST_CHECK_EQUAL(utils::Type<person_info_t>(),type_t::PERSON_INFO);
//  BOOST_CHECK_EQUAL(utils::Type<object_info_t>(),type_t::OBJECT_INFO);
}

BOOST_AUTO_TEST_SUITE_END()
