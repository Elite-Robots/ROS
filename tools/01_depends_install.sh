#!/bin/bash

info(){
    echo -e "\e[32m$1 \e[0m" 
}

error(){
    echo -e "\e[31m$1 \e[0m" 
}

debug(){
    echo -e "\e[33m$1 \e[0m" 
}

pipInstall() {
    debug "$1 will be installed"
    if ! pip3 install "$1"; then
        error "$1 install failed"
    else
        info "$1 install success"
    fi
   echo '------------------------------------------'
}

aptInstall() {
    debug "$1 will be installed"
    # if sudo apt-get install "$1" -y; then
    if ! echo "$1"; then
        error "$1 install failed"
    else
        info "$1 install success"
    fi
   echo '------------------------------------------'
   

}

aptInstall python3-pip

pipPackages=(
    elirobots
    transforms3d
    pytest
    rosdepc
)
for package in "${pipPackages[@]}"; do
    pipInstall "$package"
done

installPackages=(
    "ros-noetic-gazebo-ros-pkgs"
    "ros-noetic-controller-manager"
    "ros-noetic-joint-state-controller"
    "ros-noetic-position-controllers"
    "ros-noetic-effort-controllers"
)

for package in "${installPackages[@]}"; do
    aptInstall "$package"
done
