#! /bin/bash

toolPath=$(pwd)

wsPath=${toolPath%%tools*}

rosEnvFile=${wsPath}"devel/setup.bash"

echo "The path to the current workspace is ${wsPath}"
sleep 0.5
echo "Prepare to write environment variables"
sleep 0.5


echo -e "\n\nsource ${rosEnvFile}">> ~/.bashrc


echo "The environment variable is written successfully, please reopen the terminal to make the environment variable take effect"