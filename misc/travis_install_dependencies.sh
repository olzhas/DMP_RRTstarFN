#!/bin/bash

################################################################################
#
# Description:
#	Installs necessary dependencies for Travis-CI
#
# Authors:
#	Olzhas Adiyatov, oadiyatov@nu.edu.kz
#
# Year:
#	2016.
#
# Usage:
#	- This script is automatically called from Travis-CI script
#
################################################################################

sudo apt-get purge cmake cmake-data -y
sudo apt-get autoremove
sudo apt-get install cmake=2.8\* cmake-data=2.8\* -y

sudo apt-add-repository -y ppa:libccd-debs
sudo apt-add-repository -y ppa:fcl-debs
sudo apt-add-repository -y ppa:dartsim
sudo apt-get update -qq
sudo apt-get install -qq libeigen3-dev libdart5-dev libyaml-cpp-dev libbullet-dev libode-dev
wget -O - https://bitbucket.org/ompl/ompl/downloads/ompl-1.2.1-Source.tar.gz | tar zxf -
cd ompl-1.2.1-Source
mkdir -p build/Debug
cd build/Debug
cmake ../..
make -j4
sudo make install
#wget http://ompl.kavrakilab.org/install-ompl-ubuntu.sh && chmod u+x ./install-ompl-ubuntu.sh && ./install-ompl-ubuntu.sh

