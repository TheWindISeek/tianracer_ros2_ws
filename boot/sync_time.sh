#!/bin/bash
# 同步小车时间，直到时间差小于1秒
sudo date -s "$(ssh lucifer@192.168.1.240 date)"
# ROBOT_IP="${1:-neu@192.168.1.100}"

# while true; do
#     LOCAL_TIME=$(date +%s)
#     REMOTE_TIME=$(ssh $ROBOT_IP "date +%s" 2>/dev/null)

#     if [ -z "$REMOTE_TIME" ]; then
#         echo "无法连接到小车，重试中..."
#         sleep 2
#         continue
#     fi

#     DIFF=$((LOCAL_TIME - REMOTE_TIME))
#     DIFF=${DIFF#-}  # 取绝对值

#     echo "时间差: ${DIFF}s"

#     if [ "$DIFF" -lt 1 ]; then
#         echo "时间已同步，差值小于1秒"
#         break
#     fi

#     # 同步时间到小车
#     ssh $ROBOT_IP "sudo date -s '$(date -u +"%Y-%m-%d %H:%M:%S")'" 2>/dev/null
#     sleep 1
# done
