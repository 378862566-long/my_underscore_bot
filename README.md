# My Underscore Bot

一个用于学习 ROS 2 机器人开发的差速驱动机器人项目。

项目基于 ROS 2 Humble 和 Gazebo Fortress，使用 Xacro 描述机器人模型，
通过 `gz_ros2_control` 和 `diff_drive_controller` 控制左右轮。目前支持
自定义 Gazebo 场景、键盘速度控制、关节状态发布、TF 发布和里程计输出。

## 当前功能

- 使用 Xacro 组织机器人模型和惯性参数
- 在 Gazebo 中加载自定义 SDF 场景
- 使用 `gz_ros2_control` 连接 Gazebo 与 ROS 2 控制框架
- 使用 `diff_drive_controller` 控制左右轮
- 将键盘发布的 `/cmd_vel` 转换为控制器需要的带时间戳速度命令
- 发布 `/joint_states`、`/tf`、`/tf_static` 和里程计
- 使用自动测试检查 Xacro、launch 文件和 Gazebo 运动链路
- 使用 `rosdep` 管理 ROS 依赖
- 使用 `colcon` 构建工作空间

## 开发环境

当前版本已在以下环境中验证：

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| Gazebo | Fortress |
| 构建工具 | colcon |
| 构建类型 | ament_cmake |

## 项目结构

```text
my_underscore_bot/
├── config/
│   ├── my_controllers.yaml     # ros2_control 控制器配置
│   └── view_bot_rviz.rviz      # RViz 配置
├── description/
│   ├── robot.urdf.xacro        # 机器人模型入口
│   ├── robot_core.xacro        # 坐标基准、底盘、车轮和万向轮
│   ├── inertial_macros.xacro   # 常用形状的惯性计算宏
│   └── gazebo_control.xacro    # Gazebo ros2_control 配置
├── launch/
│   ├── launch_sim.launch.py    # Gazebo 完整仿真入口
│   └── rsp.launch.py           # robot_state_publisher 启动文件
├── scripts/
│   └── cmd_vel_relay.py        # Twist 到 TwistStamped 的速度桥接
├── test/
│   ├── test_xacro.py            # 机器人描述结构测试
│   ├── test_launch.py           # 仿真启动描述静态测试
│   └── test_motion.py           # Gazebo 运动集成测试
├── worlds/
│   └── my_world.sdf            # 当前 Gazebo 场景
├── CONTRIBUTING.md
├── LICENSE
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 安装依赖

先安装 ROS 2 Humble，并初始化 `rosdep`。然后在工作空间中执行：

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

键盘控制工具可以单独安装：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

## 构建项目

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 适合开发阶段使用。修改 Python、launch、配置和场景文件后，
安装空间会通过符号链接读取源码中的最新内容；修改 CMake 或依赖后仍应重新构建。

## 运行测试

完成构建后，运行日常测试：

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select my_underscore_bot
colcon test-result --verbose
```

日常测试包括：

- Xacro 能否展开，以及关键 link、joint 和 `ros2_control` 是否存在
- launch 文件能否加载，以及仿真参数是否声明完整
- Python、CMake、XML、版权和文档字符串等静态检查

运动测试会真实启动 Gazebo，因此日常测试默认跳过它。需要验证完整运动链路时执行：

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

RUN_GAZEBO_TESTS=1 colcon test \
  --packages-select my_underscore_bot \
  --ctest-args -R '^test_motion$' --output-on-failure
colcon test-result --verbose
```

运动测试会启动仿真、向 `/cmd_vel` 发送前进命令，并检查
`/diff_cont/odom` 中的 X 位置是否增加。测试结束后会自动关闭它启动的进程。

## 启动 Gazebo 仿真

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_underscore_bot launch_sim.launch.py
```

默认加载 `my_world.sdf`，`base_footprint` 生成在 `(0, 0, 0)`，偏航角为
`0`。`base_link` 位于 `base_footprint` 上方 `0.05 m` 的车轮中心高度。
可以通过 launch 参数选择世界并设置初始位姿：

```bash
ros2 launch my_underscore_bot launch_sim.launch.py \
  world:=my_world.sdf \
  x:=1.0 \
  y:=2.0 \
  z:=0.0 \
  yaw:=1.57
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `world` | `my_world.sdf` | `worlds/` 目录中的 Gazebo 世界文件 |
| `x` | `0.0` | 初始 X 坐标，单位米 |
| `y` | `0.0` | 初始 Y 坐标，单位米 |
| `z` | `0.0` | `base_footprint` 的初始 Z 坐标，单位米 |
| `yaw` | `0.0` | 初始偏航角，单位弧度 |

例如，`yaw:=1.57` 表示机器人初始方向旋转约 90°。

启动过程会：

1. 解析 `robot.urdf.xacro`
2. 启动 `robot_state_publisher`
3. 加载 `worlds/my_world.sdf`
4. 在 Gazebo 中生成机器人
5. 启动 `joint_state_broadcaster`
6. 启动差速驱动控制器
7. 启动速度命令桥接节点

## 键盘控制

保持 Gazebo 终端运行，打开另一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘节点发布 `geometry_msgs/msg/Twist` 类型的 `/cmd_vel`。项目中的
`cmd_vel_relay.py` 会使用控制器里程计的仿真时间戳，将它转换为
`geometry_msgs/msg/TwistStamped`，再发布到 `/diff_cont/cmd_vel`。

## 主要话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 键盘或上层节点发送速度命令 |
| `/diff_cont/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 差速控制器的速度输入 |
| `/diff_cont/cmd_vel_out` | `geometry_msgs/msg/TwistStamped` | 控制器实际采用的速度命令 |
| `/diff_cont/odom` | `nav_msgs/msg/Odometry` | 机器人里程计 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 车轮关节状态 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态坐标变换 |

当前主要 TF 关系为：

```text
odom
└── base_footprint
    └── base_link
        ├── chassis
        ├── front_caster
        ├── left_wheel
        └── right_wheel
```

## 常用检查命令

查看控制器状态：

```bash
ros2 control list_controllers
```

正常情况下应看到：

```text
joint_broad  joint_state_broadcaster/JointStateBroadcaster  active
diff_cont   diff_drive_controller/DiffDriveController       active
```

查看机器人实际采用的速度命令：

```bash
ros2 topic echo /diff_cont/cmd_vel_out
```

查看里程计：

```bash
ros2 topic echo /diff_cont/odom
```

查看所有 ROS 2 话题：

```bash
ros2 topic list
```

检查 Xacro 和 URDF：

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash

xacro src/my_underscore_bot/description/robot.urdf.xacro \
  > /tmp/my_underscore_bot.urdf
check_urdf /tmp/my_underscore_bot.urdf
```

检查世界文件：

```bash
ign sdf -k ~/dev_ws/src/my_underscore_bot/worlds/my_world.sdf
```

## 已解决的关键问题

### Gazebo 世界文件无法启动

`gz_args` 中的 `-r` 与世界文件路径之间必须有空格：

```python
launch_arguments={'gz_args': ['-r', ' ', world_file]}.items()
```

否则参数可能被拼接成：

```text
-r/home/long/...
```

Gazebo 无法识别该参数。

### 键盘命令到达话题但机器人不运动

控制器使用仿真时间判断命令是否超时。普通桥接节点最初产生了零时间戳，
导致控制器将速度命令判定为过期并清零。

当前桥接节点使用 `/diff_cont/odom` 的同源仿真时间戳生成
`TwistStamped`，保证控制器能够正确接受命令。

## 学习路线

项目将按照以下顺序继续完善：

1. 参数化世界文件和机器人初始位置（已完成）
2. 增加 Xacro、启动和运动测试（已完成）
3. 校准质量、惯性和摩擦参数
4. 添加激光雷达
5. 接入 SLAM
6. 接入 Nav2
7. 添加 IMU 和相机
8. 连接真实机器人硬件

## License

参见 [LICENSE](LICENSE)。
