project(VFOA_MODULE)
cmake_minimum_required(VERSION 2.6)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

SET(VFOA_MODULE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src"
                             "${CMAKE_CURRENT_SOURCE_DIR}/include")

SET(vfoa_module "vfoa_module")

find_package(Boost COMPONENTS date_time unit_test_framework REQUIRED)

message(STATUS "${VFOA_MODULE_INCLUDE_DIRS}")
include_directories(${VFOA_MODULE_INCLUDE_DIRS})
file(GLOB SRCS "src/*.cc")

add_library(${vfoa_module} SHARED ${SRCS})
target_link_libraries(${vfoa_module} ${Boost_LIBRARIES})

add_subdirectory(test)

#set_target_properties(${vfoa_module} PROPERTIES
#    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

install(TARGETS ${vfoa_module}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
file(GLOB headers "include/*.h")
INSTALL(FILES ${headers} DESTINATION include/vfoa_module)
