macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest.a pthread glog gtest_main.a X11)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()

package_add_test(correlative_scan_matcher_tests correlative_scan_matcher_tests.cc ../src/CorrelativeScanMatcher.cc)
