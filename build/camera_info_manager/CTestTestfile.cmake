# CMake generated Testfile for 
# Source directory: /home/shengbin/camera_ws/src/image_common/camera_info_manager
# Build directory: /home/shengbin/camera_ws/build/camera_info_manager
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cpplint "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/shengbin/camera_ws/build/camera_info_manager/test_results/camera_info_manager/cpplint.xunit.xml" "--package-name" "camera_info_manager" "--output-file" "/home/shengbin/camera_ws/build/camera_info_manager/ament_cpplint/cpplint.txt" "--command" "/opt/ros/humble/bin/ament_cpplint" "--xunit-file" "/home/shengbin/camera_ws/build/camera_info_manager/test_results/camera_info_manager/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/shengbin/camera_ws/src/image_common/camera_info_manager" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;68;ament_add_test;/home/shengbin/camera_ws/src/image_common/camera_info_manager/CMakeLists.txt;65;ament_cpplint;/home/shengbin/camera_ws/src/image_common/camera_info_manager/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/shengbin/camera_ws/build/camera_info_manager/test_results/camera_info_manager/lint_cmake.xunit.xml" "--package-name" "camera_info_manager" "--output-file" "/home/shengbin/camera_ws/build/camera_info_manager/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/shengbin/camera_ws/build/camera_info_manager/test_results/camera_info_manager/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/shengbin/camera_ws/src/image_common/camera_info_manager" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/home/shengbin/camera_ws/src/image_common/camera_info_manager/CMakeLists.txt;66;ament_lint_cmake;/home/shengbin/camera_ws/src/image_common/camera_info_manager/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/shengbin/camera_ws/build/camera_info_manager/test_results/camera_info_manager/uncrustify.xunit.xml" "--package-name" "camera_info_manager" "--output-file" "/home/shengbin/camera_ws/build/camera_info_manager/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/humble/bin/ament_uncrustify" "--xunit-file" "/home/shengbin/camera_ws/build/camera_info_manager/test_results/camera_info_manager/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/shengbin/camera_ws/src/image_common/camera_info_manager" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;70;ament_add_test;/home/shengbin/camera_ws/src/image_common/camera_info_manager/CMakeLists.txt;67;ament_uncrustify;/home/shengbin/camera_ws/src/image_common/camera_info_manager/CMakeLists.txt;0;")
