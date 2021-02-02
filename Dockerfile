FROM devrt/ros-devcontainer-vscode:melodic-desktop

RUN sudo mkdir -p /additional_software
WORKDIR /additional_software

RUN sudo apt-get update
RUN sudo apt-get upgrade -y

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

# spaCY
RUN pip install -U pip setuptools wheel
RUN pip install -U spacy
RUN python -m spacy download en_core_web_sm

# Update Python
RUN sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN sudo apt install python3-pip

# OpenCV
RUN pip3 install scikit-build
RUN pip3 install opencv-python==4.2.0.34
RUN echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc