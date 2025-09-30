# ROS2 UAV Workspace

这是一个ROS2工作空间，用于无人机（UAV）任务开发和测试。包含PX4接口、任务执行和第三方库。

## 项目结构

```
ros2_ws/
├── src/
│   ├── px4_interface/     # PX4飞控接口包
│   ├── uav_task/         # 无人机任务包
│   └── third_party/      # 第三方依赖包
│       ├── px4_msgs/     # PX4消息定义
│       ├── px4_ros_com/  # PX4 ROS通信
│       ├── BehaviorTree/ # 行为树库
│       ├── rplidar_ros/  # RPLIDAR ROS驱动
│       └── realsense-ros/# RealSense ROS驱动
├── build/                # 编译输出目录（已忽略）
├── install/              # 安装目录（已忽略）
├── log/                  # 日志目录（已忽略）
└── scripts/              # 启动脚本
```

## 依赖要求

- ROS 2 Humble Hawksbill
- PX4飞控固件
- Ubuntu 22.04 LTS

## 构建步骤

1. 安装依赖：
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-colcon-common-extensions
   ```

2. 克隆并构建工作空间：
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. 构建特定包：
   ```bash
   colcon build --packages-select uav_task
   ```

## 使用方法

### 运行无人机任务测试
```bash
ros2 launch uav_task simple_flight_test.launch.py
```

### 查看可用节点
```bash
ros2 pkg list
ros2 node list
```

## 开发指南

- 使用`colcon build`编译所有包
- 使用`source install/setup.bash`设置环境
- 日志文件位于`log/`目录
- 数据库文件`rtabmap.db`用于SLAM

## 安全提醒

⚠️ **重要安全警告**:
- 无人机测试前确保安全环境
- 检查PX4配置和传感器校准
- 保持足够的安全距离
- 建议先在仿真环境中测试

## 贡献

1. Fork项目
2. 创建功能分支
3. 提交更改
4. 发起Pull Request

## 许可证

请查看各包的许可证信息。