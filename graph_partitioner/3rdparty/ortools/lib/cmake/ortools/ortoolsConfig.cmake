## ortools CMake configuration file

set(ORTOOLS_VERSION 9.0.1)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ortoolsConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)
# Kitware CMake provide a FindZLIB.cmake module
if(NOT ZLIB_FOUND AND NOT TARGET ZLIB::ZLIB)
  find_dependency(ZLIB REQUIRED)
endif()

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.9.6")
  set(CONFIG_FLAG CONFIG)
endif()

if(NOT absl_FOUND)
  find_dependency(absl REQUIRED ${CONFIG_FLAG})
endif()
if(NOT Protobuf_FOUND AND NOT PROTOBUF_FOUND AND NOT TARGET protobuf::libprotobuf)
  find_dependency(Protobuf REQUIRED ${CONFIG_FLAG})
endif()

if(OFF)
  if(NOT scip_FOUND AND NOT TARGET libscip)
    find_dependency(scip REQUIRED ${CONFIG_FLAG})
  endif()
endif()

if(OFF)
  if(NOT Clp_FOUND AND NOT TARGET Coin::ClpSolver)
    find_dependency(Clp REQUIRED ${CONFIG_FLAG})
  endif()
  if(NOT Cbc_FOUND AND NOT TARGET Coin::CbcSolver)
    find_dependency(Cbc REQUIRED ${CONFIG_FLAG})
  endif()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/ortoolsTargets.cmake")
