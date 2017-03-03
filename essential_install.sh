#!/usr/bin/env bash
PACKAGES=""
function addpkg {
    PACKAGES="$PACKAGES $@"
}

#sudo apt-get install bc python-software-properties

addpkg\
    ros-kinetic-serial\

# go forth!
echo "apt-get install $PACKAGES"
sudo apt-get update
sudo apt-get install $PACKAGES
