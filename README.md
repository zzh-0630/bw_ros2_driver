# bw_ros_driver (ROS 2 Humble)
北微ROS驱动仓库提供了北微产品的ROS驱动程序支持，目前支持ROS1,ROS2双版本，支持串口，CAN双接口，适配北微标准协议（77开头）与Nova协议（F3开头），仓库地址如下：

ROS1：https://github.com/zzh-0630/bw_ros_driver

ROS2：https://github.com/zzh-0630/bw_ros2_driver

！！！注意：
- 本仓库完全开源，可任意修改使用
- 本仓库持续开发优化中，不保证完全可靠，若有bug，可及时联系开发人员更新维护
- ROS官方已经停止了对ROS1的维护，后续开发维护会主要在ROS2基础上进行

## 1. 功能简介

本驱动支持 **串口（Standard 0x77 / Nova 0xF3）** 与 **SocketCAN（单ID+payload mux）** 两种接入方式：

- `bw_node_standard`：解析 **0x77 标准协议**，发布
  - `sensor_msgs/msg/Imu`（`imu_topic`）
  - `sensor_msgs/msg/MagneticField`（`mag_topic`，可选）
- `bw_node_nova`：解析 **0xF3 Nova 协议**，发布同上
- `bw_node_can`：读取 **SocketCAN**，对多帧 IMU 数据进行聚合并发布 `sensor_msgs/msg/Imu`

## 2. 构建与运行（ROS 2）

### 2.1 构建

```bash
# 1) 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2) 放入本包（bw_ros_driver）
#   - 例如：git clone ...
#   - 或者把本仓库拷贝到 src 下

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2.2 串口节点（standard / nova）

1) 确认设备存在并赋权：

```bash
ls -l /dev/ttyUSB*
sudo chmod a+rw /dev/ttyUSB0
```

2) 修改参数：

- `config/common_params_ros2.yaml`

3) 启动：

```bash
ros2 launch bw_ros_driver bw_ros_standard.launch.py
# 或
ros2 launch bw_ros_driver bw_ros_nova.launch.py
```

### 2.3 CAN 节点（SocketCAN）

1) 配置 can0（bitrate 按设备实际设置）：

```bash
sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

2) 修改参数：

- `config/can_params_ros2.yaml`

3) 启动：

```bash
ros2 launch bw_ros_driver bw_ros_can.launch.py
```

## 3. 话题输出

- IMU：`/imu/data`（默认，可在参数里改）
- MAG：`/imu/mag`（串口节点默认，可在参数里改）

## 4. 常见问题

- **没有数据**：优先确认串口/波特率、CAN 接口状态（`ip link`）、以及传感器是否开启自动输出。
