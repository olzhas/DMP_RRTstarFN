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

apt-add-repository -y ppa:libccd-debs
apt-add-repository -y ppa:fcl-debs
apt-add-repository -y ppa:dartsim
apt-get update -qq
apt-get install -qq libeigen3-dev libdart5-dev
