# RViz2 地图显示问题排查指南

## 问题 1: GLSL 着色器错误

### 错误信息
```
[ERROR] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result : 
active samplers with a different type refer to the same texture image unit
```

### 原因
这是 RViz2 的已知问题，通常是由于：
- OpenGL 驱动版本兼容性问题
- 着色器版本不匹配
- 显卡驱动问题

### 解决方案

**方案 1: 忽略警告（推荐）**
- 这个错误通常**不影响地图显示**，只是警告
- 如果地图能正常显示，可以忽略

**方案 2: 更新显卡驱动**
```bash
# 检查 OpenGL 版本
glxinfo | grep "OpenGL version"

# 如果是 NVIDIA 显卡
nvidia-smi

# 更新驱动（根据你的显卡类型）
sudo apt update
sudo apt upgrade
```

**方案 3: 使用软件渲染（如果硬件加速有问题）**
```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

**方案 4: 更新 RViz2**
```bash
sudo apt update
sudo apt upgrade ros-humble-rviz2
```

---

## 问题 2: 地图位置偏离

### 症状
- 地图在 RViz2 中显示的位置与预期不符
- 地图显示在奇怪的位置
- 地图看起来被"偏移"了

### 原因分析

地图原点（origin）设置会影响地图显示位置。当前地图原点为：
```yaml
origin: [0.32, 14.5, 0]
```

这意味着：
- 地图左下角在世界坐标系中的位置是 (0.32, 14.5, 0)
- 地图尺寸：196 x 123 像素
- 分辨率：0.05 米/像素
- 实际尺寸：9.8 米 x 6.15 米

### 解决方案

#### 方案 1: 调整 RViz2 视角（最简单）

1. **移动视角**：
   - 鼠标中键拖拽：移动视角
   - 滚轮：缩放
   - 鼠标右键拖拽：旋转视角

2. **重置视角**：
   - 点击 RViz2 工具栏的 "Reset" 按钮
   - 或使用快捷键（如果有设置）

3. **适合窗口**：
   - 在 RViz2 中右键点击 3D 视图
   - 选择 "Reset" 或 "Fit to View"

#### 方案 2: 调整地图原点（如果需要）

如果地图原点设置不正确，可以修改 `my_map.yaml`：

```bash
# 编辑地图文件
nano ~/tianracer_ros2_ws/my_map.yaml
```

**选项 A: 将原点设置为 (0, 0, 0)**
```yaml
origin: [0, 0, 0]
```
这样地图左下角会在世界坐标系的原点。

**选项 B: 将原点设置为地图中心**
```yaml
# 地图中心 = (width * resolution / 2, height * resolution / 2, 0)
# = (196 * 0.05 / 2, 123 * 0.05 / 2, 0)
# = (4.9, 3.075, 0)
origin: [-4.9, -3.075, 0]
```
这样地图中心会在世界坐标系的原点。

**选项 C: 保持当前原点（如果建图时已经正确设置）**
```yaml
origin: [0.32, 14.5, 0]  # 保持原样
```
如果建图时原点已经正确设置，保持原样即可。

修改后，需要重新加载地图：
```bash
# 停止当前的 map_server
ros2 lifecycle set /map_server shutdown

# 重新启动 map_server
ros2 launch tianracer_slam_ros2 map_load.launch.py
```

#### 方案 3: 检查 Fixed Frame

确保 RViz2 的 Fixed Frame 设置为 `map`：
- 在 "Global Options" 中
- 将 "Fixed Frame" 设置为 `map`

---

## 问题 3: 地图显示不完整或模糊

### 症状
- 地图只显示一部分
- 地图看起来很模糊
- 地图颜色不对

### 解决方案

1. **检查地图分辨率**：
   ```bash
   ros2 topic echo /map --once | grep resolution
   # 应该显示: resolution: 0.05
   ```

2. **调整 RViz2 Map 显示设置**：
   - 在 Map 显示中，调整 "Alpha" 值（透明度）
   - 调整 "Color Scheme"（颜色方案）
   - 检查 "Resolution" 是否匹配（应该显示 0.05）

3. **检查地图数据**：
   ```bash
   ros2 topic echo /map --once | grep -E "width|height|resolution"
   # 应该显示正确的尺寸
   ```

---

## 快速检查清单

在 RViz2 中看不到地图或地图位置不对时，按以下顺序检查：

- [ ] Fixed Frame 设置为 `map`
- [ ] Map 显示已添加且已勾选
- [ ] Map Topic 设置为 `/map`
- [ ] QoS 设置匹配（Transient Local, Reliable）
- [ ] map_server 节点已激活（`ros2 lifecycle get /map_server` 显示 `active`）
- [ ] 地图数据存在（`ros2 topic echo /map --once` 有输出）
- [ ] TF frame `map` 存在（`ros2 run tf2_tools view_frames`）
- [ ] 尝试调整 RViz2 视角（鼠标中键拖拽、滚轮缩放）
- [ ] 尝试重新订阅地图（取消勾选 Map 显示，等待 1 秒，重新勾选）

---

## 常用命令

```bash
# 检查 map_server 状态
ros2 lifecycle get /map_server

# 检查地图数据
ros2 topic echo /map --once

# 检查 TF 树
ros2 run tf2_tools view_frames

# 检查话题信息
ros2 topic info /map -v

# 重新启动 map_server
ros2 lifecycle set /map_server shutdown
ros2 launch tianracer_slam_ros2 map_load.launch.py
```

---

**最后更新**：2026-01-18

