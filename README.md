# Elite-Robot

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

安装SDK及相关依赖

```
sudo apt install python3-pip
# 安装最新版SDK
pip install git+https://ghproxy.net/https://github.com/JunJie-zhang-o/eliterobot.git
pip install elirobots transforms3d pytest rosdepc

```

安装依赖，使用rosdepc

```
sudo rosdepc init
rosdepc update
rosdepc install --from-path src --ignore-src -y -r
```

构建代码

```
catkin_make
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

需要先修改真机IP地址，获取机器人IP方式如下：

从示教器......

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

