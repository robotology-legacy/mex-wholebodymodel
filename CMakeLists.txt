# Copyright (C) 2014 CoDyCo
# Author: Naveen Kuppuswamy
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME wholeBodyModel)
PROJECT(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# MATLAB and YARP is searched before adding the different modules because they will be used by all of them
find_package(YARP        REQUIRED)
find_package(Matlab	 REQUIRED)

if(MATLAB_FOUND)
        message(STATUS "Matlab was found!! The configuration may continue")
        set(MATLAB_LIBRARIES
                ${MATLAB_MEX_LIBRARY}
                ${MATLAB_MX_LIBRARY}
                ${MATLAB_ENG_LIBRARY}
        )
else()
        message(FATAL_ERROR "Matlab not found")
endif()
find_package(YCM                   REQUIRED)
find_package(ICUB                   REQUIRED)
find_package(Eigen3                 REQUIRED)
find_package(TinyXML                REQUIRED)
find_package(iDynTree               REQUIRED)
find_package(paramHelp              REQUIRED)
find_package(wholeBodyInterface     REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)


#### Settings for rpath
if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    if(NOT MSVC)
        #add the option to disable RPATH
        set(MEXWBI_DISABLE_RPATH FALSE CACHE BOOL "Disable RPATH installation")
        mark_as_advanced(MEXWBI_DISABLE_RPATH)
    endif(NOT MSVC)

    #Configure RPATH
    set(CMAKE_MACOSX_RPATH 1) #enable RPATH on OSX. This also suppress warnings on CMake >= 3.0
    # when building, don't use the install RPATH already
    # (but later on when installing)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    #build directory by default is built with RPATH
    set(CMAKE_SKIP_BUILD_RPATH  FALSE)

    #This is relative RPATH for libraries built in the same project
    #I assume that the directory is
    # - install_dir/something for binaries
    # - install_dir/lib for libraries
    file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_PREFIX}/bin" "${CMAKE_INSTALL_PREFIX}/lib")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
    else()
        set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
    endif()
    
    file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_PREFIX}/bin" "${CMAKE_INSTALL_PREFIX}/mex")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
    else()
        set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
    endif()

    # the RPATH to be used when installing, but only if it's not a system directory (copied form CMake File. To be tested)
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
       set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    endif("${isSystemDir}" STREQUAL "-1")

    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) #very important!

    if(MEXWBI_DISABLE_RPATH)
        #what to do? disable RPATH altogether or just revert to the default CMake configuration?
        #I revert to default
        unset(CMAKE_INSTALL_RPATH) #remove install rpath
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
    endif()
endif()
#####end RPATH


#Source and header files
#FILE(GLOB folder_source ./src/mexmain.cpp)
FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

#Some additional options
include(iCubHelpers)
include(iCubOptions)

# Check if externl torque control is to be used
if (ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL)
    message("Adding external torque control.")
    add_definitions(-DWBI_ICUB_COMPILE_PARAM_HELP)
endif()

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")


#Include directories
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${wholeBodyInterfaceIcub_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${paramHelp_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

include_directories("${MATLAB_ROOT}/simulink/include")
include_directories("${MATLAB_ROOT}/extern/include")
include_directories( ${MATLAB_INCLUDE_DIR})
include_directories( ${YARP_INCLUDE_DIRS})
include_directories( ${ICUB_INCLUDE_DIRS})

#Specify preprocessor flag for MATLAB MEX FILES Compilation.
add_definitions(-DMATLAB_MEX_FILE)

#MATLAB_MEXFILE_EXT is generated by FindMatlab.cmake
set(SFUNCTION_SUFFIX ${MATLAB_MEXFILE_EXT})

link_directories(${CMAKE_BINARY_DIR})

#Adding files used for the generation of the dynamic library
add_library(${PROJECTNAME} SHARED ${folder_source} ${folder_header})

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

#Removing default dynamic library extensions (.dll for Windows, .so for UNIX) and prefixes (lib) otherwise automatically prepended to the mex file.
set_target_properties(${PROJECTNAME} PROPERTIES SUFFIX .${SFUNCTION_SUFFIX})
set_target_properties(${PROJECTNAME} PROPERTIES PREFIX "")

#In the following specific flags will be set to ensure compliance with the default settings of Matlab's mex compiler
if(WIN32)
        if("${MATLAB_ARCH}" STREQUAL "32")
                message("Flags have been passed to compiler for Matlab 32bits on a Windows computer")
        elseif()
                message("Flags have been passed to compiler for Matlab 64bits on a Windows computer")
        endif()
                if(MSVC)
                        #Setting Compiler Flags
                        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Z7 /nologo /W3 /WX- /O2 /Oy- /D_REENTRANT /D_CRT_SECURE_NO_DEPRECATE /D_SCL_SECURE_NO_DEPRECATE /D_SECURE_SCL=0 /DyWrite_EXPORTS /Gm- /EHs /Zp8 /GS /fp:precise /Zc:wchar_t /Zc:forScope /GR /Gd /TP /showIncludes")
                        #Setting Linker Flags
                        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /INCREMENTAL:NO /NOLOGO /MANIFEST /debug /MAP.mexw64.map /MACHINE:X64 /EXPORT:mexFunction")
                        if("${MATLAB_ARCH}" STREQUAL "32")
                                #Changing some flags according to system/MATLAB version (32, 64 bits)
                                set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /MAP.mexw32.map /MACHINE:X32")
                        endif()
                elseif(MSVC)
                        message("You are not compiling using Microsoft Visual Studio, ERRORS might occur")
                endif(MSVC)
elseif(WIN32)
        message("No particular flags have been passed to compiler for Matlab 64bits on a UNIX computer")
endif(WIN32)

target_link_libraries(${PROJECTNAME}
${wholeBodyInterface_LIBRARIES}
${yarpWholeBodyInterface_LIBRARIES}
${paramHelp_LIBRARIES}
${iDynTree_LIBRARIES}
${YARP_LIBRARIES}
${MATLAB_LIBRARIES}
${skinDynLib_LIBRARIES}
${ctrlLib_LIBRARIES}
)

#set(simContext wbit)
#file(GLOB localContext ${CMAKE_CURRENT_SOURCE_DIR}/conf/wbit/*.ini)
#yarp_install(FILES       ${localContext}
#	     DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${simContext})


install(TARGETS ${PROJECTNAME} DESTINATION mex)
