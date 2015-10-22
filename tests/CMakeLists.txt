if(NOT DEFINED Matlab_MAIN_PROGRAM)
    message(FATAL_ERROR "Impossible to run tests, Matlab_MAIN_PROGRAM not found")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/addpathWBMTests.m.in
               ${CMAKE_CURRENT_BINARY_DIR}/addpathWBMTests.m)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/runWBMTests.m
               ${CMAKE_CURRENT_BINARY_DIR}/runWBMTests.m)

# Temporary workaround, add a way to load models from build directory
# as we do in iDynTree
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/twoLinks.urdf
               ${CMAKE_CURRENT_BINARY_DIR}/twoLinks.urdf)

add_test(NAME matlab_mex_wholebodymodel_tests COMMAND ${Matlab_MAIN_PROGRAM} -nodisplay -r "runWBMTests")