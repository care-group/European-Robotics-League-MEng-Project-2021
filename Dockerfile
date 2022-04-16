FROM devrt/ros-devcontainer-vscode:melodic-desktop

RUN sudo mkdir -p /additional_software

# Update key for ROS repositories
RUN sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-get clean && sudo apt-get update
RUN sudo apt-get upgrade -y

# Update Python
RUN sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN sudo apt-get install python3-pip -y
RUN sudo apt-get install python3-empy

# Required python3 modules
RUN pip3 install supervisor
RUN pip3 install supervisor_twiddler
RUN pip3 install argcomplete
RUN pip3 install rospkg==1.2.10 defusedxml netifaces
RUN pip3 install jupyter

# Manip
RUN sudo apt-get install ros-melodic-tf2-sensor-msgs

RUN sudo apt-get install -y alsa-base alsa-utils
RUN sudo apt-get install -y portaudio19-dev
RUN pip3 install SoundDevice SpeechRecognition pydub pyaudio soundfile google-cloud-texttospeech
RUN sudo apt-get install -y mpg123
#RUN chmod -R 755 /dev/snd

# spaCY
RUN pip3 install -U pip setuptools wheel
RUN pip3 install -U spacy
RUN python -m spacy download en_core_web_sm

RUN pip3 install scikit-build
RUN echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc
RUN sudo apt-get install git-lfs

# Deepface
RUN sudo apt-get update
RUN pip3 install deepface
RUN sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge -y

# Jason
RUN sudo git clone https://github.com/jason-lang/jason.git --branch v2.6 /additional_software/jason
RUN cd /additional_software/jason && sudo ./gradlew config

RUN echo 'export JASON_HOME=/additional_software/jason/build' >> ~/.bashrc
RUN echo 'export PATH=$JASON_HOME/scripts:$PATH' >> ~/.bashrc

# jason-rosbridge
RUN sudo apt-get install openjdk-8-jre -y
RUN sudo apt-get install openjdk-11-jdk -y
RUN sudo apt-get install ros-melodic-rosbridge-suite -y
RUN source /opt/ros/melodic/setup.bash

RUN sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge -y

# google cloud SDK
RUN curl -sSL https://sdk.cloud.google.com | bash

RUN pip3 install --upgrade google-cloud-speech

RUN export GOOGLE_APPLICATION_CREDENTIALS=/home/developer/workspace/src/hri/auth.json

RUN source /home/developer/google-cloud-sdk/path.bash.inc

RUN source /home/developer/google-cloud-sdk/completion.bash.inc

RUN sudo usermod -a -G video developer

RUN source /opt/ros/melodic/setup.bash
RUN echo 'source /home/developer/workspace/devel/setup.bash' >> ~/.bashrc
