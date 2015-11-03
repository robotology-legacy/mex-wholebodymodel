# Copyright (C) 2014 CoDyCo
# Author: Naveen Kuppuswamy
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

PROJECT(wholeBodyModel)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# MATLAB and YARP is searched before adding the different modules because they will be used by all of them
find_package(YARP        REQUIRED)
find_package(Matlab REQUIRED
             MX_LIBRARY
             MAIN_PROGRAM)

if(NOT Matlab_FOUND)
    message(FATAL_ERROR "Matlab not found")
endif()
find_package(YCM                   REQUIRED)
find_package(ICUB                   REQUIRED)
find_package(Eigen3                 REQUIRED)
find_package(TinyXML                REQUIRED)
find_package(iDynTree 0.3.9         REQUIRED)
find_package(paramHelp              REQUIRED)
find_package(wholeBodyInterface     REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)


#### Settings for rpath
if(NOT MSVC)
    #add the option to enable RPATH
    option(MEXWHOLEBODYMODEL_ENABLE_RPATH "Enable RPATH installation" TRUE)
    mark_as_advanced(MEXWHOLEBODYMODEL_ENABLE_RPATH)
endif(NOT MSVC)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS ${CMAKE_INSTALL_PREFIX}/bin
                          LIB_DIRS ${CMAKE_INSTALL_PREFIX}/mex ${CMAKE_INSTALL_PREFIX}/lib
                          DEPENDS MEXWHOLEBODYMODEL_ENABLE_RPATH
                          USE_LINK_PATH)


add_subdirectory(mex-wholebodymodel)


# Enable testing and generate test target
include(CTest)
enable_testing()
add_subdirectory(tests)
