# CMake generated Testfile for 
# Source directory: /home/developer/workspace/src/manip/hsrb_moveit_config
# Build directory: /workspace/build_isolated/hsrb_moveit_config
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_hsrb_moveit_config_roslaunch-check_launch "/workspace/build_isolated/hsrb_moveit_config/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/build_isolated/hsrb_moveit_config/test_results/hsrb_moveit_config/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /workspace/build_isolated/hsrb_moveit_config/test_results/hsrb_moveit_config" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/workspace/build_isolated/hsrb_moveit_config/test_results/hsrb_moveit_config/roslaunch-check_launch.xml\" -t \"/home/developer/workspace/src/manip/hsrb_moveit_config/launch\" ")
subdirs("gtest")
