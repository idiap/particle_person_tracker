file(GLOB TESTS "*.cc")

add_executable(test-exec VfoaManagerMain.cc)
add_executable(Manager_test test_manager.cc ../src/vfoa_manager.cc ../src/person.cc ../src/utils.cc ../src/types.cc ../src/vfoa_target.cc)
add_executable(Person_test test_person.cc ../src/person.cc ../src/utils.cc ../src/types.cc ../src/vfoa_target.cc)
#add_executable(Target_test test_vfoa_target.cc ../src/types.cc ../src/utils.cc ../src/vfoa_target.cc)
add_executable(Utils_test test_utils.cc ../src/types.cc ../src/utils.cc ../src/vfoa_target.cc)

target_link_libraries(test-exec vfoa_module ${Boost_DATE_TIME_LIBRARY})
target_link_libraries(Manager_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_link_libraries(Person_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#target_link_libraries(Target_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_link_libraries(Utils_test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

enable_testing()
add_test(Utils_test Utils_test)
#add_test(Target_test Target_test)
add_test(Person_test Person_test)
add_test(Manager_test Manager_test)
