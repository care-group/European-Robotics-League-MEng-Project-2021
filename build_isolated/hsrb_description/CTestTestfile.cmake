# CMake generated Testfile for 
# Source directory: /home/developer/workspace/src/manip/hsrb_description
# Build directory: /workspace/build_isolated/hsrb_description
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_hsrb_description_roslaunch-check_launch "/workspace/build_isolated/hsrb_description/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/build_isolated/hsrb_description/test_results/hsrb_description/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /workspace/build_isolated/hsrb_description/test_results/hsrb_description" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/workspace/build_isolated/hsrb_description/test_results/hsrb_description/roslaunch-check_launch.xml\" -t \"/home/developer/workspace/src/manip/hsrb_description/launch\" ")
add_test(_ctest_hsrb_description_nosetests_test "/workspace/build_isolated/hsrb_description/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/build_isolated/hsrb_description/test_results/hsrb_description/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /workspace/build_isolated/hsrb_description/test_results/hsrb_description" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/developer/workspace/src/manip/hsrb_description/test --with-xunit --xunit-file=/workspace/build_isolated/hsrb_description/test_results/hsrb_description/nosetests-test.xml")
subdirs("gtest")
