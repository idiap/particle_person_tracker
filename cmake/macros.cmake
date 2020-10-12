# Andre Anjos <andre.anjos@idiap.ch>
# 4/August/2010

#################
# BEGIN C++ macro
#################

# Add a c++ ttrack package.
#
# package: name of the c++ package
# src: list of c++ files to compile
# dependencies: list of package name this package depends on. Dependent headers
#               are automatically available for the current target. Transitivity
#               is correctly handled
# shared: additional libraries to link with.
# headers: [OPTIONAL] where to get the headers for this package from
#
# Example: ttrack_library(io "foo.cc;bar.cc" "core" "foo.so")
macro(ttrack_library package src dependencies shared)

  if(${ARGC} LESS 5)
    set(headers "${package}")
  else()
    set(headers "${ARGV4}")
  endif()

  # message(STATUS "package      '${package}'")
  #message(STATUS "src          '${src}'")
  #message(STATUS "dependencies '${dependencies}'")
  #message(STATUS "shared       '${shared}'")
  #message(STATUS "headers      '${headers}'")

  string(TOUPPER "${package}" PACKAGE)

    set(deps_list "")
    set(header_list "")
    set(compile_flags "")
    if(NOT ("${dependencies}" STREQUAL ""))
        foreach(dep ${dependencies})
            string(TOUPPER "${dep}" DEP)
            message(STATUS "TTRACK_${DEP}-NOLIB = ${TTRACK_${DEP}-NOLIB}")
            if (NOT ${TTRACK_${DEP}-NOLIB} STREQUAL "TRUE")
                list(APPEND deps_list ttrack_${dep})
            endif(NOT ${TTRACK_${DEP}-NOLIB} STREQUAL "TRUE")
            list(APPEND header_list "${TTRACK_${DEP}_HEADER_DIRS}")
        endforeach(dep)
    endif(NOT ("${dependencies}" STREQUAL ""))

    list(REMOVE_DUPLICATES header_list)

    set(TTRACK_${PACKAGE}_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${header_list} CACHE INTERNAL "${package} header dirs")

    #message(STATUS "${package} : ${TTRACK_${PACKAGE}_HEADER_DIRS}")
    #message(STATUS "${package} depslist : ${deps_list}")
    #message(STATUS "${package} hdr list : ${header_list}")

    include_directories(${TTRACK_${PACKAGE}_HEADER_DIRS})

    # include_directories(${headers})
    if(NOT ("${src}" STREQUAL ""))
        add_library(ttrack_${package} ${src})
        target_link_libraries(ttrack_${package} ${deps_list} ${shared})
        # set_target_properties(ttrack_${package} PROPERTIES LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/lib)
        if (TTRACK_SOVERSION)
            # adds versioning information
            set_target_properties(ttrack_${package} PROPERTIES VERSION ${TTRACK_VERSION})
            set_target_properties(ttrack_${package} PROPERTIES SOVERSION ${TTRACK_SOVERSION})
        endif()
        install(TARGETS ttrack_${package} EXPORT ttrack
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        set(TTRACK_${PACKAGE}-NOLIB FALSE CACHE INTERNAL "${package} has libraries")
    else(NOT ("${src}" STREQUAL ""))
        set(TTRACK_${PACKAGE}-NOLIB TRUE CACHE INTERNAL "${package} has libraries")
    endif(NOT ("${src}" STREQUAL ""))

    install(DIRECTORY ${headers} DESTINATION include/ttrack
        PATTERN ".svn" EXCLUDE
        PATTERN "*.h")
endmacro()

# Add a c++ ttrack executable.
#
# package: name of the c++ executable
# src: list of c++ files to compile
# dependencies: list of package names this package depends on. Dependent headers
#               are automatically available for the current target. Transitivity
#               is correctly handled
# shared: additional libraries to link with.
# headers: [OPTIONAL] where to get the headers for this package from
#
# Example: ttrack_executable(example "foo.cc;bar.cc" "core" "foo.so")
macro(ttrack_executable package src dependencies shared)

  if(${ARGC} LESS 5)
    set(headers "${package}")
  else()
    set(headers "${ARGV4}")
  endif()
  #message(STATUS "package      '${package}'")
  #message(STATUS "src          '${src}'")
  #message(STATUS "dependencies '${dependencies}'")
  #message(STATUS "shared       '${shared}'")
  #message(STATUS "headers      '${headers}'")

  string(TOUPPER "${package}" PACKAGE)

    set(deps_list "")
    set(header_list "")
    set(compile_flags "")
    if(NOT ("${dependencies}" STREQUAL ""))
        foreach(dep ${dependencies})
            string(TOUPPER "${dep}" DEP)
            message(STATUS "TTRACK_${DEP}-NOLIB = ${TTRACK_${DEP}-NOLIB}")
            if (NOT ${TTRACK_${DEP}-NOLIB} STREQUAL "TRUE")
                list(APPEND deps_list ttrack_${dep})
            endif(NOT ${TTRACK_${DEP}-NOLIB} STREQUAL "TRUE")
            list(APPEND header_list "${TTRACK_${DEP}_HEADER_DIRS}")
        endforeach(dep)
    endif(NOT ("${dependencies}" STREQUAL ""))

    list(REMOVE_DUPLICATES header_list)

    set(TTRACK_${PACKAGE}_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${header_list})

    message(STATUS "${package} : ${TTRACK_${PACKAGE}_HEADER_DIRS}")
    message(STATUS "${package} depslist : ${deps_list}")
    message(STATUS "${package} hdr list : ${header_list}")

    include_directories(${TTRACK_${PACKAGE}_HEADER_DIRS})

    add_executable(ttrack_${package} ${src})
    target_link_libraries(ttrack_${package} ${deps_list} ${shared})
    #set_target_properties(ttrack_${package} PROPERTIES RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/bin)
    message(STATUS "CATKIN_PACKAGE_BIN_DESTINATION=${CATKIN_PACKAGE_BIN_DESTINATION}")
    install(TARGETS ttrack_${package} EXPORT ttrack RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endmacro()

# Creates a standard TTrack test.
#
# package: package the test belongs to
# name: test name
# src: test source files
#
# Example: ttrack_test(io array test/array.cc)
macro(ttrack_test package name src shared)
  set(testname ttracktest_${package}_${name})
  message(STATUS "test      '${testname}'")
  message(STATUS "shared    '${shared}'")
  
  # Please note we don't install test executables
  add_executable(${testname} ${src})
  target_link_libraries(${testname} ttrack_${package} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY_RELEASE} ${shared})
  
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/test_reports/")
#  add_test(cxx-${package}-${name} "${CMAKE_BINARY_DIR}/bin/${testname}" --log_level=all --output_format=XML --report_level=no --log_sink=${CMAKE_BINARY_DIR}/test_reports/${testname}_report.xml)
  add_test(cxx-${package}-${name} "${CMAKE_BINARY_DIR}/bin/${testname}" --log_level=all --output_format=XML)
  set_property(TEST cxx-${package}-${name} APPEND PROPERTY ENVIRONMENT "TTRACK_TESTDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  set_property(TEST cxx-${package}-${name} APPEND PROPERTY ENVIRONMENT "TTRACK_TESTREPORT_FILE=${CMAKE_BINARY_DIR}/test_reports/${testname}_report.xml")
endmacro()

# Creates a standard TTrack benchmark.
#
# package: package the benchmark belongs to
# name: benchmark name
# src: benchmark source files
#
# Example: ttrack_benchmark(core bigtensor2d "benchmark/bigtensor2d.cc")
macro(ttrack_benchmark package name src)
  set(bindir bin)
  set(progname ttrackbench_${package}_${name})

  add_executable(${progname} ${src})
  target_link_libraries(${progname} ttrack_${package})
endmacro()

# Look for RSB dependencies to include or not RSB-dependent proxies.
#
# Example: ttrack_search_rsb()
macro(ttrack_check_rsb)
    IF (NORSB)
        message(STATUS " RSB disabled manually!")
    ELSE (NORSB)
        IF (RSC_FOUND)
            SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${RSC_CMAKE_MODULE_PATH}")
            # --- global compiler flags ---
            #INCLUDE(PedanticCompilerWarnings)
            # --- dependency handling ---
            IF (RSB_FOUND AND RSBTS_FOUND)
                IF (PROTOBUF_FOUND)
                    IF (RST_FOUND AND rst-converters_FOUND)
                        IF(NOT RSC_INTERNAL_BOOST_UUID)
                            FIND_PACKAGE(BoostUUID REQUIRED)
                        ENDIF()
                        INCLUDE_DIRECTORIES(BEFORE SYSTEM ${RSC_INCLUDE_DIRS}
                                      ${RSB_INCLUDE_DIRS}
                                      ${RST_INCLUDE_DIRS}
                                      ${BOOSTUUID_INCLUDE_DIRS}
                                      ${PROTOBUF_INCLUDE_DIR}
                                      ${RST_CONVERTERS_INCLUDE_DIRS}
                                      ${RSBTS_INCLUDE_DIRS})
    
                        message(STATUS " RSB integration defined")
                        message(STATUS " RSC include path: ${RSC_INCLUDE_DIRS}")
                        message(STATUS " RSB include path: ${RSB_INCLUDE_DIRS}")
                        message(STATUS " RST include path: ${RST_INCLUDE_DIRS}")
                        message(STATUS " BOOSTUUID include path: ${BOOSTUUID_INCLUDE_DIRS}")
                        message(STATUS " PROTOBUF include path: ${PROTOBUF_INCLUDE_DIR}")
                        message(STATUS " RST-CONVERTERS include path: ${RST_CONVERTERS_INCLUDE_DIRS}")
                        message(STATUS " RSB timesync include path: ${RSBTS_INCLUDE_DIRS}")
                        ADD_DEFINITIONS(${RST_CFLAGS})
                        ADD_DEFINITIONS(-D__RSBINTEGRATION_FOUND__)
                        message(STATUS " RST libraries ${RST_LIBRARIES} ${RSTSANDBOX_LIBRARIES}")
                        message(STATUS " RST cflags ${RST_CFLAGS} ${RSTSANDBOX_CFLAGS}")
                        set(TTRACK_RSB_INTEGRATION_FOUND TRUE
                            CACHE INTERNAL "RSB integration found flag")
                        set(TTRACK_RSB_INTEGRATION_LIBS ${RSC_LIBRARIES}
                            ${RSB_LIBRARIES}
                            ${RST_LIBRARIES}
                            ${RSTSANDBOX_LIBRARIES}
                            ${PROTOBUF_LIBRARIES}
                            ${RST_CONVERTERS_LIBRARIES}
                            ${RSBTS_LIBRARIES}
                            CACHE INTERNAL "RSB integration libraries")
                    ELSE()
                        message(STATUS " RST not found, RSB integration disabled!")
                    ENDIF()
                ELSE(PROTOBUF_FOUND)
                    message(STATUS " ProtocolBuffers not found, RSB integration disabled!")
                ENDIF(PROTOBUF_FOUND)
            ELSE()
                message(STATUS " RSB not found, RSB integration disabled!")
            ENDIF()
        ELSE(RSC_FOUND)
            message(STATUS " RSC not found, RSB integration disabled!")
        ENDIF (RSC_FOUND)    
    ENDIF (NORSB)
endmacro()


#################
# END C++ macro
#################


# Installs an example in a standard location
#
# subsys: subsystem name (currently cxx and python are supported)
# package: package of the example
# file: list of files to install as example
#
# Example: ttrack_example_install(cxx core benchmark/bigtensor2d.cc)
macro(ttrack_example_install subsys package file)
  set(exdir share/doc/ttrack/examples/${subsys}/${package})
  install(PROGRAMS ${file} DESTINATION ${exdir})
endmacro(ttrack_example_install subsys package file)


####################
# BEGIN python macro
####################

# Internal macro
# Convert a path to a python file in lib directory to a python module name
#
# package_name: package name of the file
# python_path: path to the file. Must begin with lib
# module_name: [output] converted module name
macro(ttrack_convert_file_path_to_module_name package_name python_path module_name)
  # figures out the module name from the input file dependence name
  string(REGEX REPLACE ".py$" "" ${module_name} "${python_path}")
  string(REPLACE "/" "." ${module_name} "${${module_name}}")

  if(${package_name} STREQUAL "root")
    string(REGEX REPLACE "^lib." "ttrack." ${module_name} "${${module_name}}")
  else()
    string(REGEX REPLACE "^lib." "ttrack.${package_name}." ${module_name} "${${module_name}}")
  endif()
endmacro()

# Internal macro
# Wrap a python function. Make a standalone python file which calls a python
# function.
#
# The generated script uses files build in binary tree. If file_to_install is
# not empty, another script for the install tree is generated.
#
# package_name: package name
# file_path: path to the python file containing the function. Must begin with lib.
# output_path: path where the standalone script is created.
# python_method: python function to warp (if "" set to "main")
# file_to_install: [output] path to the file to install. If empty no install file
#                  is generated.
macro(ttrack_wrap_python_file package_name file_path output_path python_method file_to_install)
  ttrack_convert_file_path_to_module_name(${package_name} ${file_path} module_name)

  set(TTRACK_MODULE ${module_name})
  if(python_method STREQUAL "")
    set(TTRACK_METHOD "main")
  else()
    set(TTRACK_METHOD "${python_method}")
  endif()

  set(TTRACK_PYTHONPATH ${CMAKE_BINARY_DIR}/${PYTHON_SITE_PACKAGES})
  configure_file(${CMAKE_SOURCE_DIR}/python/bin/wrapper.py.in ${output_path})

  if(NOT ${file_to_install} STREQUAL "")
    # Compute the temporary filename
    get_filename_component(filename ${output_path} NAME)

    # We add md5 of the full path to prevent name collision
    # The string(MD5 ...) command doesn't exists before 2.8.7
    if(CMAKE_VERSION VERSION_LESS "2.8.7")
      set(md5 "")
    else()
      string(MD5 md5 ${output_path})
      set(md5 "${md5}/")
    endif()

    set(TTRACK_MODULE ${module_name})
    if(python_method STREQUAL "")
      set(TTRACK_METHOD "main")
    else()
      set(TTRACK_METHOD "${python_method}")
    endif()

    if(IS_ABSOLUTE ${CMAKE_INSTALL_PREFIX})
      set(ABSOLUTE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
    else()
      set(ABSOLUTE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_PREFIX})
    endif()

    if(IS_ABSOLUTE ${build_path})
      set(ABSOLUTE_build_path ${build_path})
    else()
      set(ABSOLUTE_build_path ${CMAKE_BINARY_DIR}/${build_path})
    endif()

    if(IS_ABSOLUTE ${install_dir})
      set(ABSOLUTE_install_dir ${install_dir})
    else()
      set(ABSOLUTE_install_dir ${ABSOLUTE_INSTALL_PREFIX}/${install_dir})
    endif()

    set(TTRACK_PYTHONPATH ${ABSOLUTE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES})

    set(${file_to_install} ${CMAKE_BINARY_DIR}/tmp/${md5}${filename})
    configure_file(${CMAKE_SOURCE_DIR}/python/bin/wrapper.py.in ${${file_to_install}})

  endif()
endmacro()

# Creates and installs a python script (in bin directory) from a python method.
#
#   ttrack_python_script(package_name script_name file_path [python_method])
#
# package_name: package name
# script_name: name of the output script
# file_path: path to the python file containing the method.
# python_method: python method to execute by the script (default "main")
#
# Example: ttrack_python_script(ip blockDCT.py lib/script/blockDCT.py)
macro(ttrack_python_script package_name script_name file_path)
  if(${ARGC} LESS 4)
    set(python_method "main")
  else()
    set(python_method "${ARGV3}")
  endif()

  set(output_file "${CMAKE_BINARY_DIR}/bin/${script_name}")
  ttrack_wrap_python_file(${package_name} ${file_path} ${output_file} "${python_method}" file_to_install)

  # this will make the script available to the installation tree
  install(PROGRAMS ${file_to_install} DESTINATION bin)
endmacro()

# Creates and installs a python script as an example from a python method.
# In the binary tree, the script is located in bin directory. In install tree,
# the script is in the standard example directory.
#
#   ttrack_python_example(package_name script_name file_path [python_method])
#
# package_name: package name
# script_name: name of the output script
# file_path: path to the python file containing the method.
# python_method: python method to execute by the script (default "main")
#
# Example: ttrack_python_example(io video2frame.py lib/example/video2frame.py)
macro(ttrack_python_example package_name script_name file_path)
  if(${ARGC} LESS 4)
    set(python_method "main")
  else()
    set(python_method "${ARGV3}")
  endif()

  set(output_file "${CMAKE_BINARY_DIR}/bin/${script_name}")
  ttrack_wrap_python_file(${package_name} ${file_path} ${output_file} "${python_method}" "")

  ttrack_example_install(python ${package_name} ${file_path})
endmacro()

# Add python tests coded with the unittest module
#
#   ttrack_python_add_unittest(package_name file_path [python_method] [working_directory])
#
# package_name: package name
# file_path: path to the python file containing the test method
# python_method: python test method (default "main")
# working_directory: working directory where the test are executed. Default to
#                    "data" dir in current source dir.
#
# Example: ttrack_python_add_unittest(io lib/test/array.py)
macro(ttrack_python_add_unittest package_name file_path)
  if(${ARGC} LESS 3)
    set(python_method "main")
  else()
    set(python_method "${ARGV2}")
  endif()

  # The 4th parameter is optional, it indicates the cwd for the test
  if(${ARGC} LESS 4)
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data")
      set(cwd "${CMAKE_CURRENT_SOURCE_DIR}/data")
    else()
      set(cwd "")
    endif()
  else()
    set(cwd "${ARGV3}")
  endif()

  string(REGEX REPLACE ".py$" "" test_filename "${file_path}")
  string(REPLACE "/" "." test_filename "${test_filename}")
  string(REGEX REPLACE "^lib." "ttrack.${package_name}." test_filename "${test_filename}")
  string(REPLACE "." "_" test_filename "${test_filename}")
  set(output_file "${CMAKE_BINARY_DIR}/bin/test_python_${test_filename}.py")

  ttrack_wrap_python_file(${package_name} ${file_path} ${output_file} "${python_method}" "")

  get_filename_component(test_name_suffix ${file_path} NAME_WE)
  set(test_name_suffix "${test_name_suffix}-${python_method}")
  string(REPLACE "." "_" test_name "python-${package_name}-${test_name_suffix}")

  if(cwd STREQUAL "")
    add_test(${test_name} ${output_file} --verbose --verbose)
  else()
    add_test(${test_name} ${output_file} --cwd=${cwd} --verbose --verbose)
  endif()

  # Common properties to all tests
  set_property(TEST ${test_name} APPEND PROPERTY ENVIRONMENT "TTRACK_TESTDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data")
endmacro()

# Configure the wrapper to the python binary that automatically sets the
# correct environment.
#
# Warning: this macro must be call only once per project.
#
#  ttrack_configure_ttrackpython(file_input build_path install_dir)
#
# file_input: path to the script to configure
# script_name: the destination name for the script
#
# Example: ttrack_configure_ttrackpython(bin/python.in bin/python)
macro(ttrack_configure_ttrackpython file_input script_name executable)

  get_filename_component(install_dir ${script_name} PATH)
  get_filename_component(install_name ${script_name} NAME)

  if ("${install_name}" STREQUAL "") 
    set(install_name ".") 
  endif()

  # configures and installs the build directory version of the script
  set(ABSOLUTE_build_path ${CMAKE_BINARY_DIR})
  set(TTRACK_PYTHONPATH ${CMAKE_BINARY_DIR}/${PYTHON_SITE_PACKAGES})
  configure_file(${file_input} ${ABSOLUTE_build_path}/${script_name} @ONLY)

  # gets the absolute installation prefix
  if(IS_ABSOLUTE ${CMAKE_INSTALL_PREFIX})
    set(ABSOLUTE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  else()
    set(ABSOLUTE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_PREFIX})
  endif()

  # configures and installs the installation directory version of the script
  set(ABSOLUTE_install_dir ${ABSOLUTE_INSTALL_PREFIX}/${install_dir})
  set(TTRACK_PYTHONPATH ${ABSOLUTE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES})
  configure_file(${file_input} ${CMAKE_BINARY_DIR}/tmp/${install_name}.toinstall @ONLY)
  if("${executable}" STREQUAL "ON")
    install(PROGRAMS ${CMAKE_BINARY_DIR}/tmp/${install_name}.toinstall DESTINATION ${install_dir} RENAME ${install_name})
  else()
    install(FILES ${CMAKE_BINARY_DIR}/tmp/${install_name}.toinstall DESTINATION ${install_dir} RENAME ${install_name})
  endif()

endmacro()

##################
# END python macro
##################

# Internal macro.
# Recursively copy files from a folder to the build tree and install them. The
# macro respects the relative path of the file.
# Warning: this macro only generate custom command to copy the files. To really
# copy the files you need to have a target that depends on these files.
#
#  copy_files(input_dir include_regex exclude_regex output_dir install_dir output_files target program install_exclude_paths)
#
# input_dir: directory with the files to copy
# include_regex: globbing expressions for the included files
# exclude_regex: globbing expressions for the excluded files
# output_dir: directory to copy the files (path relative to CMAKE_BINARY_DIR)
# install_dir: directory to install the files, if "" no file is installed. (path
#              relative to CMAKE_INSTALL_PREFIX)
# output_files: [output] list of file generated by the commands
# target: related target (only used to display state)
# install_exclude_paths: list of regex applies to the relative input path. Files
#                        that match these regex are not installed.
#
macro(copy_files input_dir include_regex exclude_regex output_dir install_dir output_files target program install_exclude_paths)
  set(input_files "")
  foreach(exp ${include_regex})
    file(GLOB_RECURSE files RELATIVE "${input_dir}" "${input_dir}/${exp}")
    list(APPEND input_files ${files})
  endforeach()

  foreach(exp ${exclude_regex})
    file(GLOB_RECURSE files RELATIVE "${input_dir}" "${input_dir}/${exp}")
    list(REMOVE_ITEM input_files "${files}")
  endforeach()

  set(${output_files} "")
  foreach(input_file_rel ${input_files})
    set(input_file "${input_dir}/${input_file_rel}")
    set(output_file "${output_dir}/${input_file_rel}")

    add_custom_command(OUTPUT "${output_file}"
                       DEPENDS "${input_file}"
                       COMMAND ${CMAKE_COMMAND} -E copy "${input_file}" "${output_file}"
                       COMMENT "Copying ${input_file_rel} for ${target}")
                       #COMMENT "") ## Use this one to remove output text

    set(install_exclude FALSE)
    foreach(install_exclude_path ${install_exclude_paths})
      if(${input_file_rel} MATCHES ${install_exclude_path})
        set(install_exclude TRUE)
        break()
      endif()
    endforeach()

    if (NOT install_dir STREQUAL "" AND NOT install_exclude)
      get_filename_component(rel_path ${input_file_rel} PATH)
      if (program)
        install(PROGRAM ${input_file} DESTINATION "${install_dir}/${rel_path}")
      else()
        install(FILES ${input_file} DESTINATION "${install_dir}/${rel_path}")
      endif()
    endif()

    list(APPEND ${output_files} "${output_file}")
  endforeach()
endmacro()

# Internal macro.
# Add a new python package. The target automatically depends on the
# corresponding c++ package and his python binding.
#
#   ttrack_python_bindings(cxx_package package cxx_src pydependencies)
#
# cxx_package: corresponding c++ package
# package: name of the python package
# cxx_src: c++ source for the package
# pydependencies: list of additional python package dependencies
macro(ttrack_python_bindings cxx_package package cxx_src pydependencies)
  if(${ARGC} LESS 5)
    set(subpackage "FALSE")
  else()
    set(subpackage "${ARGV4}")
  endif()

  string(TOUPPER "${package}" PACKAGE)
  string(TOUPPER "${cxx_package}" CXX_PACKAGE)

  set(pydeps_list "ttrack_${cxx_package}")
  set(pyheader_list "")
  if(NOT ("${pydependencies}" STREQUAL ""))
    foreach(dep ${pydependencies})
      string(TOUPPER "${dep}" DEP)
      list(APPEND pydeps_list ttrack_${dep})
      list(APPEND pyheader_list "${TTRACK_${DEP}_HEADER_DIRS}")
    endforeach(dep)
  endif(NOT ("${pydependencies}" STREQUAL ""))

  list(REMOVE_DUPLICATES pyheader_list)

  set(TTRACK_PYTHON_${PACKAGE}_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${TTRACK_${CXX_PACKAGE}_HEADER_DIRS} ${pyheader_list} CACHE INTERNAL "${package} header dirs")
  include_directories(${TTRACK_PYTHON_${PACKAGE}_HEADER_DIRS} ${python_INCLUDE_DIRS})
  message(STATUS "Python ${package}:")
  message(STATUS "${pydependencies}")
  message(STATUS "${pyheader_list} !! ${pydeps_list}")
  message(STATUS "${package}/${cxx_package} : ${TTRACK_PYTHON_${PACKAGE}_HEADER_DIRS} - ${TTRACK_${CXX_PACKAGE}_HEADER_DIRS}")

  message(STATUS " Python SWIG binding, package ${package}, sources ${cxx_src}")
  
  if("${cxx_src}" STREQUAL "")
    add_custom_target(pyttrack_${package} ALL)
    ## TODO Add correct dependencies

  else()
  
    ## TODO: 1) add support for additional sources
    ## TODO: 2) add support for multiple ".i" files
  
    set(swig_other_sources "")
    set(swig_sources "")
    
    foreach(src ${cxx_src})
        list(APPEND swig_sources "${CMAKE_CURRENT_SOURCE_DIR}/${src}")
    endforeach(src ${cxx_src})
  
    set(swig_generated_file_name ${cxx_package}_wrap.cxx)
    
    list(APPEND SWIG_ARGUMENTS "-python" "-c++")
    list(APPEND SWIG_ARGUMENTS -o "${swig_generated_file_name}")
    foreach(incdir ${TTRACK_PYTHON_${PACKAGE}_HEADER_DIRS})
        list(APPEND SWIG_ARGUMENTS -I"${incdir}")
    endforeach(incdir)
    list(APPEND SWIG_ARGUMENTS "${swig_sources}")
    string (REPLACE ";" " " SWIG_ARGUMENTS_STR "${SWIG_ARGUMENTS}")
    message(STATUS "Swig arguments: ${SWIG_ARGUMENTS}")
    
    add_custom_command(
        OUTPUT "${swig_generated_file_name}"
        COMMAND "${SWIG_EXECUTABLE}"
        ARGS ${SWIG_ARGUMENTS}
        DEPENDS "${cxx_src}"
        COMMENT "Swig source")
    set_source_files_properties("${swig_generated_file_name}"
        PROPERTIES GENERATED 1)
    add_custom_target(pyttrack_${package}_swig_files DEPENDS ${swig_generated_file_name})
        
    set(target_name "pyttrack_${package}")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
    message(STATUS "Included current source directory ${CMAKE_CURRENT_SOURCE_DIR}/src")
    
    add_library(${target_name}
        MODULE
        ${swig_generated_file_name}
        ${swig_other_sources})
    add_dependencies(${target_name} pyttrack_${package}_swig_files)
    
    
    target_link_libraries(${target_name} ${pydeps_list})
    set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "${package}")
    set_target_properties(${target_name} PROPERTIES PREFIX "_")
    set_target_properties(${target_name} PROPERTIES SUFFIX ".so")
    set(pycxx_flags "-Wno-long-long -Wno-unused-function")
    set_target_properties(${target_name} PROPERTIES COMPILE_FLAGS ${pycxx_flags})
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_SITE_PACKAGES}/ttrack/${cxx_package})

#    string(REPLACE "_" "/" package_path ${package})
    set(package_path ${package})

    # makes sure bindings are installed at the right places
    # install(TARGETS ${target_name} LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}/ttrack/${package_path})
    message(STATUS "${target_name} install to ${PYTHON_SITE_PACKAGES}/ttrack/${package_path}")

  endif()

  # Install scripts only if not a subpackage
  if (NOT subpackage)
    if(CXX_PACKAGE STREQUAL "ROOT")
      set(bin_path "${CMAKE_BINARY_DIR}/${PYTHON_SITE_PACKAGES}/ttrack")
      set(install_path "${PYTHON_SITE_PACKAGES}/ttrack")
    else()
      set(bin_path "${CMAKE_BINARY_DIR}/${PYTHON_SITE_PACKAGES}/ttrack/${cxx_package}")
      set(install_path "${PYTHON_SITE_PACKAGES}/ttrack/${cxx_package}")
    endif()

    set(input_dir "${CMAKE_CURRENT_SOURCE_DIR}/lib")

    # Copy python files from lib folder
    copy_files("${input_dir}" "*.py" "*~;.*.swp;.swp*;*.pyc;*.in;.svn;.svn/*" ${bin_path}
                ${install_path} output_lib_files "pyttrack_${package}" FALSE "^test;^example")

    file(GLOB_RECURSE files RELATIVE "${input_dir}" "${input_dir}/*.in")
    foreach(file ${files})
      string(REGEX REPLACE "\\.in$" "" outputfile "${file}")
      configure_file("${input_dir}/${file}" "${bin_path}/${outputfile}" @ONLY)
      install(FILES "${bin_path}/${outputfile}"
              DESTINATION ${install_path})
    endforeach()

    add_custom_target(pyttrack_${package}_files DEPENDS ${output_lib_files} ${output_script_files})
    add_dependencies(pyttrack_${package} pyttrack_${package}_files)
  endif()

  if(NOT TARGET pyttrack_compile_python_files)
    add_custom_target(pyttrack_compile_python_files ALL COMMAND ${PYTHON_EXECUTABLE} -m compileall -q "${CMAKE_BINARY_DIR}/${PYTHON_SITE_PACKAGES}/ttrack")
  endif()

  add_dependencies(pyttrack_compile_python_files pyttrack_${package})

endmacro()

# Add a new python package binding c++ code. The target automatically depends on
# the corresponding c++ package and his python binding.
#
#   ttrack_python_package_bindings(package cxx_src pydependencies)
#
# package: name of the python package (should be the same as the corresponding
#          cxx package)
# cxx_src: c++ source for the package
# pydependencies: list of additional python package dependencies
#
# Example: ttrack_python_package_bindings(io "src/foo.cc;src/bar.cc" core_array)
macro(ttrack_python_package_bindings package cxx_src pydependencies)
  ttrack_python_bindings("${package}" "${package}" "${cxx_src}" "${pydependencies}" FALSE)
endmacro()

# Add a new python subpackage binding c++ code. The target automatically depends
# on the corresponding c++ package and his python binding.
#
#   ttrack_python_subpackage_bindings(package subpackage cxx_src pydependencies)
#
# package: name of the python package (should be the same as the corresponding
#          cxx package)
# subpackage: name of the python subpackage
# cxx_src: c++ source for the package
# pydependencies: list of additional python package dependencies
#
# Example: ttrack_python_subpackage_bindings(core array "src/foo.cc" "")
macro(ttrack_python_subpackage_bindings package subpackage cxx_src pydependencies)
  ttrack_python_bindings("${package}" "${package}_${subpackage}" "${cxx_src}" "${pydependencies}" TRUE)
  set_target_properties(pyttrack_${package}_${subpackage} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_SITE_PACKAGES}/ttrack/${package}/${subpackage})
endmacro()


# This macro helps users to add python tests to cmake
function(ttrack_python_add_test)

  list(GET ARGV 0 test_name)
  list(GET ARGV 1 prog)
  list(REMOVE_AT ARGV 0) #pop from front
  list(REMOVE_AT ARGV 0) #pop from front

  get_filename_component(prog_filename ${prog} NAME)

  # temporary hack to get the other tests working
  if ("${prog}" STREQUAL "${prog_filename}")
    # new style testing
    get_filename_component(prog_filename_we ${prog} NAME_WE)
    set(test_name "python-${test_name}-${prog_filename_we}")
    add_test(${test_name};${CMAKE_BINARY_DIR}/bin/${prog_filename};${ARGV})
  else()
    # TODO: get rid of this once all tests have been migrated
    add_test(${test_name};${prog};${ARGV})
  endif()

  # Common properties to all python tests
  set_property(TEST ${test_name} APPEND PROPERTY ENVIRONMENT "TTRACK_TESTDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data")

endfunction()

# This macro helps users to add python tests to cmake that depends on other
# tests
function(ttrack_python_add_dependent_test)

  list(GET ARGV 0 test_name)
  list(GET ARGV 1 dependencies)
  list(GET ARGV 2 prog)
  list(REMOVE_AT ARGV 0) #pop from front
  list(REMOVE_AT ARGV 0) #pop from front
  list(REMOVE_AT ARGV 0) #pop from front

  ttrack_python_add_test(${test_name};${prog};${ARGV})

  get_filename_component(prog_filename_we ${prog} NAME_WE)
  set(test_name "python-${test_name}-${prog_filename_we}")

  set_property(TEST ${test_name} APPEND PROPERTY DEPENDS "${dependencies}")

endfunction()
