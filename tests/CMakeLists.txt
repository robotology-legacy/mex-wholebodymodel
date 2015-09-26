configure_file(${CMAKE_CURRENT_SOURCE_DIR}/runWBMTests.m
               ${CMAKE_CURRENT_BINARY_DIR}/runWBMTests.m)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/addpathWBMTests.m.in
               ${CMAKE_CURRENT_BINARY_DIR}/addpathWBMTests.m)

add_test(NAME matlab_mex_wholebodymodel_tests COMMAND ${Matlab_MAIN_PROGRAM} -nodisplay -r "runWBMTests")