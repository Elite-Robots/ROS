# Elite-Robot-ROS

- 系统平台：Ubuntu20.04 
- ROS版本：Noetic

## 文件组织结构

```
.
├── elite_controller		# 艾利特机器人ros控制器,当控制真实机器人时使用			
├── elite_description		# 机器人描述文件功能包
├── elite_driver			# 机器人的驱动程序功能包
├── elite_gazebo			# gazebo仿真功能包
├── elite_moveit			# moveit功能包
├── elite_msgs				# 自定义消息功能包
└── moveit_config			# moveIt Config 功能包
```

## 1.安装编译

### 1.1 安装依赖及构建

#### 安装SDK及相关依赖

```
sudo apt install python3-pip

# 安装最新版SDK
pip3 install elirobots transforms3d pytest rosdepc

sudo rosdepc init

rosdepc update
```

#### Elite Robots ROS开发包下载与编译

```
# 创建工作空间目录
mkdir ~/catkin_ws && cd ~/catkin_ws

# 克隆Elite ROS 功能包
git clone git@github.com:Elite-Robots/ROS.git

cd ROS

# 依赖安装
rosdepc install --from-path src --ignore-src -y -r

# 编译
catkin_make
```

### 1.2 工具脚本的使用

额外的，我们还提供了几个工具脚本的使用，以使您尽量的缩短在环境部署上的时间

```
└── tools
    ├── 00_fishros_install.sh
    ├── 01_depends_install.sh
    └── 02_source_rosws_env.sh
```

### 00_auto_install.sh

该脚本是一个开源的自动安装ros以及相关环境配置的脚本，通过该脚本可以一键安装ROS，一键安装VSCode，一键更换系统源以及一键配置rosdep等多种功能。开源地址：https://github.com/fishros/install

#### 使用方法

```
cd tools
./00_auto_install.sh
```

### 01_depends_install.sh

在使用ROS平台的相关软件时，会出现一些由于软件包未安装导致运行时出现的报错，基于已有的一些软件包，我们制作了一个一键安装的脚本。如果您再使用中发现有一些其他的软件包我们并未添加进去，可以通过issues或邮件告知我们，我们第一时间进行更新。

#### 使用方法

```
cd tools
./01_depends_install.sh
```

### 02_source_rosws_env.sh

在使用ros时，需要在终端中频繁的将ros的工作空间加入环境变量中，通过该脚本，将当前的工作空间自动写入环境变量中，后续无需频繁的`source devel/setup.bashrc`.

#### 使用方法

```
cd tools
./02_source_rosws_env.sh
```

## 2.启动

目前支持三种模式，虚拟模式，仿真模式和真机模式。

- 虚拟模式，不需要真实机械臂和仿真环境，可以快速验证Moveit相关程序。
- 仿真模式，通过Moveit控制仿真机械臂，可以不依赖真实机械臂完成仿真验证。
- 真机模式，通过Moveit直接控制真实机械臂。

### 2.1.虚拟模式

虚拟模式只需要启动moveit本身及自带虚拟机械臂。

```
source devel/setup.bash
roslaunch elite_moveit elite_moveit.launch robot:=ec66 mode:=fake 
```
### 2.2.仿真模式

仿真模式需要先启动仿真环境再启动Moveit控制程序。

启动仿真程序，ec66为例，可将ec63更换为ec63、ec612

```
source devel/setup.bash
roslaunch elite_gazebo elite_gazebo.launch robot:=ec66
```

启动MoveIt，ec66为例，可将ec63更换为ec63、ec612

```
source devel/setup.bash
roslaunch elite_moveit elite_moveit.launch robot:=ec66 mode:=sim 
```

### 2.3.真机模式

需要先修改真机IP地址，获取机器人IP方式如下：https://bbs.elibot.cn/forum/detail/topic/87.html

保证电脑可以通过ping指令与真实机械臂相互连接后，将真机IP地址作为参数传入launch指令中`robot_ip:=192.168.1.201 `

启动真实机械臂控制器，ec66为例，可将ec63更换为ec63、ec612

```
source devel/setup.bash
roslaunch elite_controller elite_controller.launch robot:=ec66 robot_ip:=192.168.1.201 
```

启动MoveIt，ec66为例，可将ec63更换为ec63、ec612

```
source devel/setup.bash
roslaunch elite_moveit elite_moveit.launch robot:=ec66 mode:=real 
```

## 问题反馈

如果你发现了任何错误，请联系我们！！！

论坛：https://bbs.elibot.cn/

邮箱：elite@elibot.cn
