# tianbot_mini_webots

tianbot_mini webots仿真功能包，是为TianbotMini ROS移动机器人学习平台专门搭建的webots仿真平台功能包。

## 使用指南
[ROS联合webots实战案例目录](https://blog.csdn.net/xiaokai1999/article/details/112601720)
### 部署功能包
- 命令行进入`catkin_ws/src`
``` shell
$ cd catkin_ws/src
$ git clone https://github.com/JackyMao1999/tianbot_mini_webots
```
- 加载`tianbotmini`模型功能包
``` shell
$ cd catkin_ws/src
$ git clone https://github.com/tianbot/tianbot_mini_description
```
- 编译功能包
``` shell
$ cd catkin_ws/
$ catkin_make
```
### 1.启动机器人仿真环境
``` shell
$ roslaunch tianbot_mini_webots webots.launch
```
### 2.启动机器人控制程序过程
``` shell
$ roslaunch tianbot_mini_webots webots.launch
$ rosrun tianbot_mini_webots tianbot_velocity
```
### 3.启动机器人gmapping建图功能过程
``` shell
$ roslaunch tianbot_mini_webots webots.launch
$ roslaunch tianbot_mini_webots slam_with_gmapping.launch
```
#### 3.1启动机器人导航功能过程
``` shell
$ rosrun tianbot_mini_webots tianbot_2dnav_move
```
注意：在机器人导航过程中`tianbot_2dnav_move`和`tianbot_velocity`程序只能开其中一个
### 4.趣味应用:机器人视觉巡线刷圈
教程：[ROS联合Webots之实现趣味机器人巡线刷圈](https://blog.csdn.net/xiaokai1999/article/details/118637666)
``` shell
$ roslaunch tianbot_mini_webots webots.launch
$ rosrun tianbot_mini_webots tianbot_velocity  ## 注意：需要先启动一下然后退出
$ rosrun tianbot_mini_webots tianbot_line_follow  ## 启动巡线程序
```