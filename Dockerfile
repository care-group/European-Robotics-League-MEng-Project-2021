FROM devrt/ros-devcontainer-vscode:melodic-desktop

RUN sudo mkdir -p /additional_software
WORKDIR /additional_software

# Update key for ROS repositories
RUN sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt clean && sudo apt update
RUN sudo apt-get upgrade -y

# Update Python
RUN sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN sudo apt install python3-pip -y

# Required python3 modules
RUN pip3 install supervisor
RUN pip3 install supervisor_twiddler
RUN pip3 install argcomplete

# spaCY
RUN pip3 install -U pip setuptools wheel
RUN pip3 install -U spacy
RUN python -m spacy download en_core_web_sm

# OpenCV
RUN pip3 install scikit-build
RUN pip3 install opencv-python==4.2.0.34
RUN echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc

# Deepface
RUN pip3 install deepface

# Jason
RUN sudo git clone https://github.com/jason-lang/jason.git
RUN cd jason && sudo ./gradlew config

RUN echo 'export JASON_HOME=/additional_software/jason/build' >> ~/.bashrc
RUN echo 'export PATH=$JASON_HOME/scripts:$PATH' >> ~/.bashrc

# jason-rosbridge
RUN sudo apt-get install openjdk-8-jre -y
RUN sudo apt-get install openjdk-11-jdk -y
RUN sudo apt-get install ros-melodic-rosbridge-suite -y
RUN source /opt/ros/melodic/setup.bash
