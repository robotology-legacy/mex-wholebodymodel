# Copyright (C) 2016 CoDyCo
# Author: Gabriele Nava
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.8)

PROJECT(mexWholeBodyModel)

# Look for dependencies
find_package(Matlab REQUIRED
                    MX_LIBRARY
                    MAIN_PROGRAM)

if(NOT Matlab_FOUND)
    message(FATAL_ERROR "Matlab not found")
endif()

find_package(ICUB                   REQUIRED)
find_package(YARP                   REQUIRED)
find_package(YCM                    REQUIRED)
find_package(Eigen3                 REQUIRED)
find_package(TinyXML                REQUIRED)
find_package(iDynTree 0.3.9         REQUIRED)
find_package(wholeBodyInterface     REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

include(YCMDefaultDirs)
include(YarpInstallationHelpers)
include(AddInstallRPATHSupport)

yarp_configure_external_installation(codyco)

### Settings for rpath
if(NOT MSVC)
    #add the option to enable RPATH
    option(MEXWHOLEBODYMODEL_ENABLE_RPATH "Enable RPATH installation" TRUE)
    mark_as_advanced(MEXWHOLEBODYMODEL_ENABLE_RPATH)
endif(NOT MSVC)

add_install_rpath_support(BIN_DIRS ${CMAKE_INSTALL_PREFIX}/bin
                          LIB_DIRS ${CMAKE_INSTALL_PREFIX}/mex ${CMAKE_INSTALL_PREFIX}/lib
                          DEPENDS MEXWHOLEBODYMODEL_ENABLE_RPATH
                          USE_LINK_PATH)

# Enable testing and generate test target
include(CTest)
enable_testing()

# Subdirectories
add_subdirectory(mex-wholebodymodel)
add_subdirectory(tests)

# STARTUP mexWholeBodyModel
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/.startup_mexWholeBodyModel.m.in ${CMAKE_BINARY_DIR}/startup_mexWholeBodyModel.m)
install(FILES ${CMAKE_BINARY_DIR}/startup_mexWholeBodyModel.m DESTINATION mex)


