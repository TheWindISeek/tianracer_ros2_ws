#!/usr/bin/bash

# ROS2 通信诊断脚本
# 用于排查小车和电脑之间的 ROS2 通信问题

echo "=========================================="
echo "ROS2 通信诊断工具"
echo "=========================================="
echo ""

# 1. 检查 ROS2 环境
echo "1. 检查 ROS2 环境变量:"
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-未设置（默认0）}"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-未设置（默认FastDDS）}"
echo "   FASTRTPS_DEFAULT_PROFILES_FILE: ${FASTRTPS_DEFAULT_PROFILES_FILE:-未设置}"
echo ""

# 2. 检查网络接口
echo "2. 检查网络接口:"
ip -4 addr show | grep -E "^[0-9]+:|inet " | grep -v "127.0.0.1" | head -10
echo ""

# 3. 检查话题列表
echo "3. 检查当前可用的话题:"
if command -v ros2 &> /dev/null; then
    echo "   正在获取话题列表..."
    ros2 topic list 2>&1 | head -20
    echo ""
    
    # 检查图像相关话题
    echo "4. 检查图像相关话题:"
    ros2 topic list 2>&1 | grep -i -E "(image|camera)" || echo "   未找到图像相关话题"
    echo ""
    
    # 检查话题信息
    echo "5. 检查图像话题详细信息（如果有）:"
    IMAGE_TOPICS=$(ros2 topic list 2>&1 | grep -i -E "(image|camera)" | head -3)
    if [ -n "$IMAGE_TOPICS" ]; then
        for topic in $IMAGE_TOPICS; do
            echo "   话题: $topic"
            ros2 topic info $topic 2>&1 | head -5
            echo "   消息频率:"
            timeout 2 ros2 topic hz $topic 2>&1 | head -3 || echo "     无法获取频率（可能没有发布者）"
            echo ""
        done
    else
        echo "   未找到图像话题"
    fi
    echo ""
    
    # 检查节点
    echo "6. 检查相机相关节点:"
    ros2 node list 2>&1 | grep -i -E "(camera|realsense)" || echo "   未找到相机节点"
    echo ""
    
    # 检查 DDS 参与者
    echo "7. 检查 DDS 发现信息:"
    echo "   使用 ros2 daemon info 查看 DDS 参与者信息..."
    ros2 daemon info 2>&1 | head -20
    echo ""
else
    echo "   错误: 未找到 ros2 命令，请先 source source.sh"
    exit 1
fi

# 8. 检查 FastDDS 配置
echo "8. 检查 FastDDS 配置文件:"
if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "   配置文件存在: $FASTRTPS_DEFAULT_PROFILES_FILE"
    echo "   配置文件内容:"
    cat "$FASTRTPS_DEFAULT_PROFILES_FILE" | head -20
else
    echo "   警告: FastDDS 配置文件不存在或未设置"
fi
echo ""

# 9. 网络连接建议
echo "9. 网络连接建议:"
echo "   - 确保小车和电脑在同一网络（或可以互相访问）"
echo "   - 检查防火墙是否阻止了多播流量（UDP 端口 7400-7500）"
echo "   - 确保 ROS_DOMAIN_ID 在小车和电脑上相同"
echo "   - 如果使用 WiFi，确保多播功能已启用"
echo ""

# 10. 快速测试命令
echo "10. 建议的测试命令:"
echo "   在小车上运行: ros2 topic echo /camera/color/image_raw"
echo "   在电脑上运行: ros2 topic list"
echo "   在电脑上运行: ros2 topic hz /camera/color/image_raw"
echo ""

echo "=========================================="
echo "诊断完成"
echo "=========================================="

