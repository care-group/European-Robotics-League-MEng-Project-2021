#!/bin/bash
cd ..
mkdir additional_software
cd additional_software

sudo apt-get update
sudo apt-get upgrade

# Jason Install
git clone https://github.com/jason-lang/jason.git
cd jason
./gradlew config

if !(grep -Fxq "# Jason Paths" ~/.bashrc)
then
    echo "# Jason Paths" >> ~/.bashrc
    echo "export JASON_HOME=/additional_software/jason/build" >> ~/.bashrc
    echo 'export PATH=$JASON_HOME/scripts:$PATH' >> ~/.bashrc
    source ~/.bashrc
fi

# jason-rosbridge
sudo apt-get install openjdk-8-jre
sudo apt-get install ros-melodic-rosbridge-suite
source /opt/ros/melodic/setup.bash
cd ..
