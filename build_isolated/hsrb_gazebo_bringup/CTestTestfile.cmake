# CMake generated Testfile for 
# Source directory: /home/developer/workspace/src/manip/hsrb_gazebo_bringup
# Build directory: /workspace/build_isolated/hsrb_gazebo_bringup
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_hsrb_gazebo_bringup_roslaunch-check_launch "/workspace/build_isolated/hsrb_gazebo_bringup/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/build_isolated/hsrb_gazebo_bringup/test_results/hsrb_gazebo_bringup/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /workspace/build_isolated/hsrb_gazebo_bringup/test_results/hsrb_gazebo_bringup" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/workspace/build_isolated/hsrb_gazebo_bringup/test_results/hsrb_gazebo_bringup/roslaunch-check_launch.xml\" -t \"/home/developer/workspace/src/manip/hsrb_gazebo_bringup/launch\" ")
subdirs("gtest")
