# Git Submodules 使用指南

本项目支持两种方式管理依赖：

1. **ROS2 repos 文件方式**（推荐）- 使用 `tianracer_ros2_ws.repos` 和 `vcs import`
2. **Git 子模块方式** - 使用 Git 原生子模块功能

## 使用 Git 子模块方式

### 初始化子模块

如果你已经克隆了仓库，需要初始化子模块：

```bash
# 初始化并克隆所有子模块
git submodule update --init --recursive
```

### 添加子模块（首次设置）

如果你想要将现有的依赖转换为子模块，运行：

```bash
./init_submodules.sh
```

这个脚本会：
- 清理现有的嵌入 git 仓库
- 将所有依赖添加为 Git 子模块
- 使用 `.repos` 文件中指定的分支和 URL

### 克隆包含子模块的仓库

```bash
# 方法1: 克隆时同时初始化子模块
git clone --recursive git@github.com:TheWindISeek/tianracer_ros2_ws.git

# 方法2: 先克隆，再初始化子模块
git clone git@github.com:TheWindISeek/tianracer_ros2_ws.git
cd tianracer_ros2_ws
git submodule update --init --recursive
```

### 更新子模块

```bash
# 更新所有子模块到最新提交
git submodule update --remote

# 更新特定子模块
git submodule update --remote src/slam_toolbox
```

### 切换到特定版本

```bash
# 进入子模块目录
cd src/slam_toolbox

# 切换到特定标签或提交
git checkout 2.6.10

# 返回主仓库
cd ../..

# 提交子模块版本更新
git add src/slam_toolbox
git commit -m "[user][update]: update slam_toolbox to 2.6.10"
```

### 子模块列表

当前项目包含以下子模块：

- `src/ackermann_msgs` - Ackermann drive message definitions (ros2 分支)
- `src/librealsense` - Intel RealSense SDK (master 分支)
- `src/osight_lidar_ros2` - LiDAR driver (dev 分支)
- `src/realsense-ros` - Intel RealSense ROS2 driver (ros2-development 分支)
- `src/slam_toolbox` - SLAM toolkit (master 分支)
- `src/tianbot_core_ros2` - Core communication package (dev 分支)

### 常用命令

```bash
# 查看子模块状态
git submodule status

# 查看子模块信息
cat .gitmodules

# 移除子模块（如果需要）
git submodule deinit -f src/package_name
git rm -f src/package_name
rm -rf .git/modules/src/package_name
```

## 两种方式对比

| 特性 | ROS2 repos 文件 | Git 子模块 |
|------|----------------|-----------|
| 工具 | vcstool | git 原生 |
| 版本控制 | 提交哈希 | 提交哈希 |
| 分支管理 | 支持 | 支持 |
| 更新方式 | `vcs pull` | `git submodule update --remote` |
| 克隆方式 | `vcs import` | `git clone --recursive` |
| 推荐场景 | ROS2 项目 | 纯 Git 项目 |

## 注意事项

1. **不要混用两种方式**：选择一种方式后，建议保持一致
2. **子模块是独立的仓库**：每个子模块都有自己的 `.git` 目录
3. **提交子模块更新**：修改子模块后需要在主仓库中提交引用更新
4. **分支管理**：子模块默认会跟踪指定的分支，但可以切换到特定提交

