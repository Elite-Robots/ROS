# Elite-Robot-ROS

- System Platform: Ubuntu20.04 
- ROS Version: Noetic

## File Structure

```
.
├── elite_controller		# Elite Robot ros Controller, for controlling the robot.
├── elite_description		# Function Package for Robot Decription Files
├── elite_driver			# Function Package for Robot Driver Files
├── elite_gazebo			# Function Package for gazebo Simulation
├── elite_moveit			# Function Package for moveit
├── elite_msgs				# Function Package for Custome Message
└── moveit_config			# Function Package for moveIt Config
```

## 1.Installation and Compilation

### 1.1 Install Dependency and Construction

#### Install SDK and Related Dependencies

```
sudo apt install python3-pip

# Install the newest SDK
pip3 install elirobots transforms3d pytest rosdepc

sudo rosdepc init

rosdepc update
```

####  Download and Compile Development package for Elite Robots ROS

```
# Create Work Space Index
mkdir ~/catkin_ws && cd ~/catkin_ws

# Copy Elite ROS Function Package
git clone git@github.com:Elite-Robots/ROS.git

cd ROS

# Install Dependencies
rosdepc install --from-path src --ignore-src -y -r

# Compile
catkin_make
```

### 1.2 Use Tool Scripts

In addition, several tool scripts are provided for minimize environment deployment.

```
└── tools
    ├── 00_fishros_install.sh
    ├── 01_depends_install.sh
    └── 02_source_rosws_env.sh
```

### 00_auto_install.sh

This script is an open source script for automatically installing ROS and related environment configuration. Through this script, the user can install ROS, install vscode, change system source and configure rosdep with one click. Open source address: https://github.com/fishros/install

#### How to use

```
cd tools
./00_auto_install.sh
```

### 01_depends_install.sh

When using the related software of the ROS platform, there will be some errors during operation due to missing some of the software package. Based on some existing software packages, we have made a one click installation script. If you find some other software packages that still missing, please inform us through issues or email, and we will update them as soon as possible.

#### How to use

```
cd tools
./01_depends_install.sh
```

### 02_source_rosws_env.sh

When using ROS, the workspace of ROS need to be frequently added to the environment variable in the terminal. Through this script, the current workspace is automatically written into the environment variable, so the code of `source devel/setup.bashrc` will no longer needed frequently.

#### How to use

```
cd tools
./02_source_rosws_env.sh
```

## 2. Launching

Currently supports three modes: virtual mode, simulation mode and real machine mode.

- Virtual Mode: does not need a real robot arm and simulation environment, and can quickly verify Moveit related programs. 
- Simulation Mode: By controlling the simulated manipulator with Moveit, the simulation verification can be completed without relying on the real robot.
- Real Machine Mode: Directly control the real robot through Moveit.

### 2.1. Virtual Mode

The virtual mode only needs to start Moveit and its own virtual robot arm.

```
source devel/setup.bash
roslaunch elite_moveit elite_moveit.launch robot:=ec66 mode:=fake 
```
### 2.2. Simulation Mode

The simulation mode needs to start the simulation environment before starting the Moveit control program.

Start the simulation program, taking EC66 as an example, it can be change to EC63 or EC612

```
source devel/setup.bash
roslaunch elite_gazebo elite_gazebo.launch robot:=ec66
```

Start Moveit, taking EC66 as an example, it can be change to EC63 or EC612

```
source devel/setup.bash
roslaunch elite_moveit elite_moveit.launch robot:=ec66 mode:=sim 
```

### 2.3. Real Machine Mode

Modify the IP address to match the actual robot. Guide for obtain the IP address of the robot: https://bbs.elibot.cn/forum/detail/topic/87.html

After ensuring that the computer can be connected with the real robot arm through the ping command, the IP address of the real machine is passed into the launch command as a parameter `robot_ip:=192.168.1.201 `

Start the real robot controller, taking EC66 as an example, it can be change to EC63 or EC612

```
source devel/setup.bash
roslaunch elite_controller elite_controller.launch robot:=ec66 robot_ip:=192.168.1.201 
```

Start Moveit, taking EC66 as an example, it can be change to EC63 or EC612

```
source devel/setup.bash
roslaunch elite_moveit elite_moveit.launch robot:=ec66 mode:=real 
```

## Feedbacks

If you find any error, please contact us!!!

BBS: https://bbs.elibot.cn/

Email: elite@elibot.cn