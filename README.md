# husky man 使用文档

## 概述

由husky室外移动机器人，两台kinova 6d2f机械臂，2自由度云台，Kinect v2深度摄像机，组成移动双臂机器人，主要用于完成定位导航与识别抓取任务。

## 功能包简介
- husky_app: 遥操作控制、移动抓取demo
- husky_man_bringup: 启动程序
- husky_man_moveit_config: Moveit机械臂规划
- husy_man_rviz: Rviz 显示
- husky_man_description:  机器人描述模型

## 环境构建

```
    # 创建工作空间
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src/

    # 拷贝功能包
    https://github.com/I-Quotient-Robotics/husky_man_robot.git
    
    # 使用rosdep安装依赖
    sudo apt-get update
    sudo apt-get install python-rosdep
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src	--ignore-src -i -y

    # 构建功能包
    cd ~/catkin_ws
    catkin_make 
```

## 基础操作

1. 上电开机
   - 将钥匙旋转到打开状态
   - 旋起急停按钮
   - 按下底盘电源开关
2. 关机断电
   - 用手拖住机械臂，按下机械臂电源按钮，将机械臂放置在husky上，注意磕碰
   - 关闭husky电源按钮
3. 电池充电
   - 打开电池盖，拔下电源接口
   - 将充电插头插入电池插口，充电时间大概4小时
   - 充电完成后插回机身接口，盖上电源盖

## 快速启动

### 启动基础功能

1. 启动

   ```shell
   # 启动底盘、左臂、云台、摄像头、手柄控制
   roslaunch husky_man_bringup bringup.launch
   # 启动右臂
   roslaunch husky_man_bringup kinova_bringup.launch
   ```

### 创建地图

1. 启动建图，使用手柄控制机器人移动进行地图的创建

   ```shell
   roslaunch husky_navigation gmapping.launch
   ```

2. 保存地图

   ```shell
   # map为文件名
   rosrun map_server map_saver -f map
   ```

### 定位导航

 1. 启动amcl定位，move_base导航

    ```shell
    roslaunch husky_navigation amcl_demo.launch
    ```

### Moveit！

1. 启动moveit控制

   ```shell
   roslaunch husky_man_moveit_config excute.launch
   ```

## 遥操作

#### 手柄控制

1. 启动，默认base中已启动

   ```shell
   roslaunch husky_man_app teleop.launch
   ```
2. 键位图
   ![avatar](https://github.com/I-Quotient-Robotics/husky_man_ros/blob/master/pic/joy_forward.png)

   ![avatar](https://github.com/I-Quotient-Robotics/husky_man_ros/blob/master/pic/joy_back.png)

3. 使用说明
   - 初始启动后，需回到home位，才能使用其他功能
   - 回到home使用API，没有避障功能，需要在离home较近的位置使用

#### 键盘控制

1. 启动

   ```shell
   roslaunch husky_man_app keyboard.launch
   ```

2. 键位设置

   | 键位（左臂/右臂）       | 功能                                             |
   | :---------------------- | :----------------------------------------------- |
   | W  S/I  K               | 末端在Y方向的前进后退或者旋转                    |
   | A  D/J  I               | 末端在Z方向的前进后退或者旋转                    |
   | Q  R/U  O               | 末端在X方向的前进后退或者旋转                    |
   | Z  X/N  M               | 控制模式选择，末端平移与旋转                     |
   | V  B/M  N               | 机械臂控制移动速度增加与减少                     |
   | 1 2 3 4 5 6/= - 0 9 8 7 | 1到6关节控制                                     |
   | ～                      | 按住并同时转动关节，提供反方向的关节运动         |
   | Enter                   | 暂停键盘控制，默认关闭，按一次开启，再次按下关闭 |
   | Esc                     | 暂定driver，默认开启，按一次关闭，再次按下开启   |
   | R  F/Y H                | 夹爪开启与闭合                                   |
   | T/G                     | 回到Home位                                       |
   | UP DOWN                 | husky在x方向的速度                               |
   | LEFT RIGHT              | husky在z方向的旋转速度                           |
   | PaUp PgDn               | 移动速度增加与减少                               |

3. 使用说明

   - 键盘控制需要在keyboard的小窗口聚焦状态

## 移动抓取

1. 启动

   ```shell
   roslaunch husky_man_app box_detect.launch
   ```

2. 参照 *快速启动* 教程完成建图。

3. 修改导航加载地图：修改husky_navigation下amcl_demo.launch中map名称。

4. 设置导航点：
    4.1 关闭建图相关节点，启动导航节点，初始化位置，控制机器人移动到预设位置。
    4.2 查看amcl_pose消息，得到机器人位姿
```shell
    rostopic echo /amcl_pose
```

    4.3 在husky_man_app.yaml中修改导航点,Standby：home点；pick：抓取点；plack：放置点

5. 注意：
    5.1 导航点不能离障碍物太近，并且在移动抓取中机械臂的臂长有限，导航点距离桌子的距离需要调整（不同桌高距离不同）
    5.2 目前无法在规划过程中规避除机器人本身的障碍物，在使用过程中，应尽量设置多个路径点来规避障碍物，若出非理想规划过程，或者有几率碰到障碍物，应及时按下手柄stop按钮，或者快速断电

## 注意事项

1. husky急停按钮会切断除自身外所有电源，机械臂本身无断电抱闸，应尽量避免使用此功能
2. 机器人充电时间大概为4小时，使用时间4小时，若突然断电，需注意接住机械臂，避免机械臂掉落磕碰

## 相关功能包
以下为相关功能包的简介与链接
- [aruco_ros](https://github.com/pal-robotics/aruco_ros)： 基于二维码的姿态识别
- [Kinect V2](https://github.com/code-iai/iai_kinect2)： Kinect相机ROS包
- [husky_robot](https://github.com/husky/husky)：husky移动机器人ROS包
- [iqr_pan_tilt](https://github.com/I-Quotient-Robotics/iqr_pan_tilt)： 云台ROS包
- [kinova-ros](https://github.com/Kinovarobotics/kinova-ros)： kinova机械臂ROS包
