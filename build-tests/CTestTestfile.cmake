# CMake generated Testfile for 
# Source directory: /home/szy/work/robot/tripod/axvisor_two/aka-rk3588
# Build directory: /home/szy/work/robot/tripod/axvisor_two/aka-rk3588/build-tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(feetech_motion_policy "/home/szy/work/robot/tripod/axvisor_two/aka-rk3588/build-tests/feetech_motion_policy_test")
set_tests_properties(feetech_motion_policy PROPERTIES  _BACKTRACE_TRIPLES "/home/szy/work/robot/tripod/axvisor_two/aka-rk3588/CMakeLists.txt;18;add_test;/home/szy/work/robot/tripod/axvisor_two/aka-rk3588/CMakeLists.txt;0;")
add_test(bucket_detector "/home/szy/work/robot/tripod/axvisor_two/aka-rk3588/build-tests/bucket_detector_test")
set_tests_properties(bucket_detector PROPERTIES  _BACKTRACE_TRIPLES "/home/szy/work/robot/tripod/axvisor_two/aka-rk3588/CMakeLists.txt;27;add_test;/home/szy/work/robot/tripod/axvisor_two/aka-rk3588/CMakeLists.txt;0;")
subdirs("utils.out")
