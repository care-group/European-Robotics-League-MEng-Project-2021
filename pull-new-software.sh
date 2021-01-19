#!/bin/sh

cd ..
mkdir additional_software
cd additional_software

# Jason Install
git clone https://github.com/jason-lang/jason.git
cd jason
./gradlew config
export JASON_HOME=$/additional_software/jason/build
export PATH=$JASON_HOME/scripts:$PATH
cd ..