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

sudo apt-add-repository -y ppa:libccd-debs
sudo apt-add-repository -y ppa:fcl-debs
sudo apt-add-repository -y ppa:dartsim
sudo apt-get update -qq
sudo apt-get install -qq libeigen3-dev libdart5-dev libyaml-cpp-dev
ls /usr/share/cmake-2.8/Modules/
