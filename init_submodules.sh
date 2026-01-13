#!/usr/bin/bash

# 初始化 Git 子模块脚本
# 根据 tianracer_ros2_ws.repos 文件中的信息初始化所有子模块

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}初始化 Git 子模块${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查是否在 git 仓库中
if [ ! -d .git ]; then
    echo -e "${RED}错误: 当前目录不是 git 仓库${NC}"
    echo "请先运行: git init"
    exit 1
fi

# 检查并询问是否移除现有的 src 目录中的嵌入仓库
found_embedded_repos=false
embedded_repos=()

if [ -d src/ackermann_msgs/.git ]; then
    found_embedded_repos=true
    embedded_repos+=("src/ackermann_msgs")
fi
if [ -d src/librealsense/.git ]; then
    found_embedded_repos=true
    embedded_repos+=("src/librealsense")
fi
if [ -d src/osight_lidar_ros2/.git ]; then
    found_embedded_repos=true
    embedded_repos+=("src/osight_lidar_ros2")
fi
if [ -d src/realsense-ros/.git ]; then
    found_embedded_repos=true
    embedded_repos+=("src/realsense-ros")
fi
if [ -d src/slam_toolbox/.git ]; then
    found_embedded_repos=true
    embedded_repos+=("src/slam_toolbox")
fi
if [ -d src/tianbot_core_ros2/.git ]; then
    found_embedded_repos=true
    embedded_repos+=("src/tianbot_core_ros2")
fi

if [ "$found_embedded_repos" = true ]; then
    echo -e "${YELLOW}检测到以下目录包含嵌入的 git 仓库:${NC}"
    for repo in "${embedded_repos[@]}"; do
        echo -e "  - ${YELLOW}$repo${NC}"
    done
    echo ""
    echo -e "${YELLOW}这些目录需要移除 .git 文件夹才能添加为子模块。${NC}"
    read -p "是否继续删除这些 .git 文件夹? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}操作已取消${NC}"
        exit 1
    fi
    
    echo -e "${YELLOW}正在清理嵌入的 git 仓库...${NC}"
    for repo in "${embedded_repos[@]}"; do
        if [ -d "$repo/.git" ]; then
            rm -rf "$repo/.git"
            echo -e "  ${GREEN}已移除: $repo/.git${NC}"
        fi
    done
    echo ""
fi

# 添加子模块
echo -e "${GREEN}添加 Git 子模块...${NC}"
echo ""

# ackermann_msgs
echo -e "${BLUE}添加 ackermann_msgs...${NC}"
git submodule add -b ros2 git@github.com:ros-drivers/ackermann_msgs.git src/ackermann_msgs

# librealsense
echo -e "${BLUE}添加 librealsense...${NC}"
git submodule add -b master https://github.com/IntelRealSense/librealsense.git src/librealsense

# osight_lidar_ros2
echo -e "${BLUE}添加 osight_lidar_ros2...${NC}"
git submodule add -b dev git@github.com:TheWindISeek/osight_lidar_ros2.git src/osight_lidar_ros2

# realsense-ros
echo -e "${BLUE}添加 realsense-ros...${NC}"
git submodule add -b ros2-development https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros

# slam_toolbox
echo -e "${BLUE}添加 slam_toolbox...${NC}"
git submodule add -b master git@github.com:SteveMacenski/slam_toolbox.git src/slam_toolbox

# tianbot_core_ros2
echo -e "${BLUE}添加 tianbot_core_ros2...${NC}"
git submodule add -b dev git@github.com:TheWindISeek/tianbot_core_ros2.git src/tianbot_core_ros2

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}子模块添加完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}注意:${NC}"
echo "1. 子模块已添加到 .gitmodules 文件"
echo "2. 要初始化所有子模块，运行: git submodule update --init --recursive"
echo "3. 要更新所有子模块，运行: git submodule update --remote"
echo "4. 要克隆包含子模块的仓库，运行: git clone --recursive <repo-url>"
echo ""

