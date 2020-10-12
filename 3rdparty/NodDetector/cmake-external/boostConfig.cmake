# Tries to find a local version of Boost installed
# Andre Anjos - 02.july.2010

FIND_PACKAGE(Boost REQUIRED system unit_test_framework thread filesystem date_time
  program_options  serialization system regex)


# set(Requested_Boost_VERSION "1.40.0")

# if(boost_FIND_REQUIRED)
#     set(REQUIRED_FLAG "REQUIRED")
# endif(boost_FIND_REQUIRED)

# # Compiles against mt versions
# set(Boost_USE_MULTITHREADED ON)

# # Determine here the components you need so the system can verify
# set(_BOOST_COMPONENTS unit_test_framework thread filesystem date_time
#     program_options signals serialization system regex)
# if (TTRACK_BUILD_PYTHON)
#     list(APPEND _BOOST_COMPONENTS python)
# endif ()
# find_package(Boost ${Requested_Boost_VERSION} ${REQUIRED_FLAG}
#     COMPONENTS ${_BOOST_COMPONENTS}
# )

# IF (Boost_FOUND)
#     if (TTRACK_BUILD_PYTHON)
#         list(APPEND boost_INCLUDE_DIRS ${python_INCLUDE_DIRS})
#         list(APPEND boost_LIBRARY_DIRS ${python_LIBRARY_DIRS})
#         list(APPEND boost_LIBRARIES ${PYTHON_LIBRARY})
#     endif ()

#     # Renaming so all works automagically
#     set(boost_INCLUDE_DIRS ${Boost_INCLUDE_DIRS} CACHE INTERNAL
#         "Boost include directories")
#     include_directories(SYSTEM ${boost_INCLUDE_DIRS})
#     set(boost_LIBRARY_DIRS ${Boost_LIBRARY_DIRS} CACHE INTERNAL
#         "Boost library directories")
#     set(boost_LIBRARIES ${Boost_LIBRARIES} CACHE INTERNAL
#         "Boost libraries")

#     message(STATUS "BOOST include directories: ${Boost_INCLUDE_DIRS}")
#     message(STATUS "BOOST library directories: ${Boost_LIBRARY_DIRS}")
#     message(STATUS "BOOST libraries: ${Boost_LIBRARIES}")
# ELSE(Boost_FOUND)
#     message(STATUS "BOOST not found!")
# ENDIF(Boost_FOUND)

# if(boost_FIND_REQUIRED)
#     unset(REQUIRED_FLAG)
# endif(boost_FIND_REQUIRED)
