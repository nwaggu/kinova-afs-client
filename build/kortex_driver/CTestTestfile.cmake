# CMake generated Testfile for 
# Source directory: /home/sharer/official_kortex_prime/src/ros_kortex/kortex_driver
# Build directory: /home/sharer/official_kortex_prime/build/kortex_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_kortex_driver_gtest_kortex_arm_driver_func_tests "/home/sharer/official_kortex_prime/build/kortex_driver/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sharer/official_kortex_prime/build/kortex_driver/test_results/kortex_driver/gtest-kortex_arm_driver_func_tests.xml" "--return-code" "/home/sharer/official_kortex_prime/devel/.private/kortex_driver/lib/kortex_driver/kortex_arm_driver_func_tests --gtest_output=xml:/home/sharer/official_kortex_prime/build/kortex_driver/test_results/kortex_driver/gtest-kortex_arm_driver_func_tests.xml")
set_tests_properties(_ctest_kortex_driver_gtest_kortex_arm_driver_func_tests PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/sharer/official_kortex_prime/src/ros_kortex/kortex_driver/CMakeLists.txt;172;catkin_add_gtest;/home/sharer/official_kortex_prime/src/ros_kortex/kortex_driver/CMakeLists.txt;0;")
subdirs("gtest")
