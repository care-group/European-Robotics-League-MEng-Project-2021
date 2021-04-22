git lfs pull
git update-index --assume-unchanged vgg_face_weights.h5
git update-index --assume-unchanged src/cv/yolo/yolov3.weights

# `python-catkin-tools` is needed for catkin tool
# `python3-dev` and `python3-catkin-pkg-modules` is needed to build cv_bridge
# `python3-numpy` and `python3-yaml` is cv_bridge dependencies
# `ros-kinetic-cv-bridge` is needed to install a lot of cv_bridge deps. Probaply you already have it installed.
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge
python -m pip uninstall opencv-python -y
python -m pip uninstall opencv-contrib-python -y
python -m pip install opencv-contrib-python

# Move the deepface weights to the appropriate directory
mkdir /home/developer/.deepface
mkdir /home/developer/.deepface/weights
cp vgg_face_weights.h5 /home/developer/.deepface/weights/vgg_face_weights.h5

catkin clean -y

rm src/manip/hsrb_moveit_config/config/kinematics.yaml
cp src/manip/kinematics.yaml src/manip/hsrb_moveit_config/config/kinematics.yaml

# Create catkin workspace
catkin init
# Instruct catkin to set cmake variables
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
# Instruct catkin to install built packages into install place. It is $CATKIN_WORKSPACE/install folder
catkin config --no-install
# Clone cv_bridge src
git clone https://github.com/ros-perception/vision_opencv.git src/cv/vision_opencv
# Find version of cv_bridge in your repository
apt-cache show ros-melodic-cv-bridge | grep Version
    Version: 1.12.8-0xenial-20180416-143935-0800
# Checkout rcvight version in git repo. In our case it is 1.12.8
cd src/cv/vision_opencv/
git checkout melodic
cd ../../../
# Build
catkin build
