# Configures and builds utils module
# Vasil Khalidov - 10 May 2012

project(ttrack-cxx-utils)
cmake_minimum_required(VERSION 2.6)

# Dependencies of this package

set(deps "") # other TTrack subprojects
set(shared "")

file(GLOB src "src/*.cc")

INCLUDE_DIRECTORIES(SYSTEM ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
LIST(APPEND shared ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${external_LIBS})

ttrack_library(utils "${src}" "${deps}" "${shared}")


#file(GLOB SRCS "*.cc")
#
#set(CMAKE_CXX_FLAGS "-g -Wall -fPIC")
#
#add_library(vfoa_utils SHARED ${SRCS})
#target_link_libraries(vfoa_utils ${OPENCV_LIBRARIES} ${Boost_LIBRARIES})
#
#INSTALL(TARGETS vfoa_utils
#        LIBRARY DESTINATION lib)
