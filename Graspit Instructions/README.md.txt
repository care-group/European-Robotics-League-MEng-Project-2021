README.md

Before executing the bash script ensure that the SoQt tarboll file [soqt-1.6.0-src.tar.gz]
is downloaded [https://github.com/coin3d/soqt/releases] to the src directory.
You may need to check the directories and ensure the architecture is suitable for your system
as this was developed remotely using a docker container but all commands for required libraries and packages
should remain the same.

Bash script is commented to help making necessary changes a simpler process.

NOTE: It is expected that "workspace" is a setup ROS workspace.
NOTE: Coin is a large library and will take a while to install.

### Links to resources used ###
Coin install instructions [https://github.com/coin3d/coin/blob/master/INSTALL]
SoQt install instructions [https://github.com/iat-cener/tonatiuh/wiki/Installing-SoQt-For-Linux]
graspit_interface install instructions [https://github.com/graspit-simulator/graspit_interface]
GraspIt! homepage [https://graspit-simulator.github.io/]