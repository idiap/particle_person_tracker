# This include file only defines which dependencies we need, globally
set(NODDETECTOR_DEPENDENCIES boost fftw gsl)
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(NODDETECTOR_DEPENDENCIES python ${NODDETECTOR_DEPENDENCIES})
endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
set(NODDETECTOR_OPTIONALS )
