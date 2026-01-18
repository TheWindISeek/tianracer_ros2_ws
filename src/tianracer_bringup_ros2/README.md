# Tianracer Bringup Package

这个包提供了一个统一的launch文件，可以一次性启动三个核心节点：
- Osight Lidar (激光雷达)
- Realsense Camera (深度相机)
- Tianbot Core (机器人核心控制)

## 使用方法

### 基本启动（使用默认参数）

```bash
ros2 launch tianracer_bringup tianracer_bringup.launch.py
```

### 自定义参数启动

#### Lidar 参数

```bash
ros2 launch tianracer_bringup tianracer_bringup.launch.py \
    lidar_ip:=192.168.1.10 \
    lidar_name:=osight_lidar \
    lidar_model:=iexxx \
    lidar_frame_id:=laser \
    lidar_angle_max:=2.3562 \
    lidar_angle_min:=-2.3562
```

#### Realsense 参数

```bash
ros2 launch tianracer_bringup tianracer_bringup.launch.py \
    realsense_camera_name:=camera \
    realsense_camera_namespace:=camera \
    realsense_serial_no:="''" \
    realsense_enable_color:=true \
    realsense_enable_depth:=true
```

#### Tianbot Core 参数

```bash
ros2 launch tianracer_bringup tianracer_bringup.launch.py \
    tianbot_serial_port:=/dev/tianbot_racecar \
    tianbot_serial_baudrate:=460800 \
    tianbot_type:=ackermann \
    tianbot_type_verify:=true \
    tianbot_publish_tf:=true
```

### 组合使用示例

```bash
ros2 launch tianracer_bringup tianracer_bringup.launch.py \
    lidar_ip:=192.168.1.10 \
    tianbot_serial_port:=/dev/tianbot_racecar \
    tianbot_type:=ackermann \
    realsense_enable_color:=true \
    realsense_enable_depth:=true
```

## 参数说明

### Lidar 参数
- `lidar_ip`: 激光雷达IP地址 (默认: 192.168.1.10)
- `lidar_name`: 激光雷达节点名称 (默认: osight_lidar)
- `lidar_model`: 激光雷达型号 (默认: iexxx)
- `lidar_frame_id`: 激光雷达frame ID (默认: laser)
- `lidar_angle_max`: 最大扫描角度 (默认: 2.3562)
- `lidar_angle_min`: 最小扫描角度 (默认: -2.3562)

### Realsense 参数
- `realsense_camera_name`: 相机名称 (默认: camera)
- `realsense_camera_namespace`: 相机命名空间 (默认: camera)
- `realsense_serial_no`: 相机序列号，空字符串表示自动选择 (默认: '')
- `realsense_config_file`: Realsense配置文件路径 (默认: '')
- `realsense_enable_color`: 启用彩色流 (默认: true)
- `realsense_enable_depth`: 启用深度流 (默认: true)

### Tianbot Core 参数
- `tianbot_serial_port`: 串口设备路径 (默认: /dev/tianbot_racecar)
- `tianbot_serial_baudrate`: 串口波特率 (默认: 460800)
- `tianbot_type`: 机器人类型: omni, diff, ackermann (默认: ackermann)
- `tianbot_type_verify`: 是否验证设备类型 (默认: true)
- `tianbot_publish_tf`: 是否发布TF变换 (默认: true)

## 注意事项

1. 确保所有依赖包已正确安装和编译
2. 根据实际硬件配置调整参数
3. 如果Realsense需要更多高级配置，可以使用`realsense_config_file`参数指定YAML配置文件
4. 串口设备路径需要根据实际系统调整

