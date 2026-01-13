#!/bin/bash
# sync_workspace.sh - 灵活的代码同步脚本（支持交互式密码输入和双向同步）

# 默认配置（可通过命令行参数覆盖）
REMOTE_USER="${REMOTE_USER:-neu}"
REMOTE_HOST="${REMOTE_HOST:-}"
REMOTE_PATH="${REMOTE_PATH:-~/tianracer_ros2_ws}"
LOCAL_PATH="${LOCAL_PATH:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示使用说明
usage() {
    echo -e "${BLUE}使用方法: $0 [选项] [命令]${NC}"
    echo ""
    echo -e "${YELLOW}命令:${NC}"
    echo "  push     - 将本地代码推送到远程（默认）"
    echo "  pull     - 从远程拉取代码到本地"
    echo "  dry-run  - 预览将要同步的文件（不实际同步）"
    echo "  specific - 同步指定的包（需要提供包名）"
    echo ""
    echo -e "${YELLOW}选项:${NC}"
    echo "  -h, --host HOST       远程主机地址（IP或主机名）"
    echo "  -u, --user USER       远程用户名（默认: neu）"
    echo "  -p, --path PATH       远程路径（默认: ~/tianracer_ros2_ws）"
    echo "  --help                显示此帮助信息"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  $0 push -h 192.168.1.100              # 推送代码到指定IP"
    echo "  $0 pull -h neu-desktop -u neu         # 从远程拉取代码"
    echo "  $0 dry-run -h 192.168.1.100           # 预览同步"
    echo "  $0 specific tianbot_core_ros2 -h 192.168.1.100  # 只同步指定包"
    echo ""
    echo -e "${YELLOW}环境变量:${NC}"
    echo "  也可以通过环境变量设置:"
    echo "  REMOTE_USER=neu REMOTE_HOST=192.168.1.100 $0 push"
}

# 解析命令行参数
parse_args() {
    local args=("$@")
    local i=0
    
    while [[ $i -lt ${#args[@]} ]]; do
        case "${args[$i]}" in
            -h|--host)
                if [[ $((i+1)) -lt ${#args[@]} ]]; then
                    REMOTE_HOST="${args[$((i+1))]}"
                    i=$((i+2))
                else
                    echo -e "${RED}错误: -h/--host 需要指定主机地址${NC}"
                    exit 1
                fi
                ;;
            -u|--user)
                if [[ $((i+1)) -lt ${#args[@]} ]]; then
                    REMOTE_USER="${args[$((i+1))]}"
                    i=$((i+2))
                else
                    echo -e "${RED}错误: -u/--user 需要指定用户名${NC}"
                    exit 1
                fi
                ;;
            -p|--path)
                if [[ $((i+1)) -lt ${#args[@]} ]]; then
                    REMOTE_PATH="${args[$((i+1))]}"
                    i=$((i+2))
                else
                    echo -e "${RED}错误: -p/--path 需要指定路径${NC}"
                    exit 1
                fi
                ;;
            --help)
                usage
                exit 0
                ;;
            *)
                # 不是选项，停止解析
                break
                ;;
        esac
    done
}

# 检查必需的工具
check_dependencies() {
    if ! command -v rsync &> /dev/null; then
        echo -e "${RED}错误: 未找到 rsync，请安装: sudo apt-get install rsync${NC}"
        exit 1
    fi
    
    if ! command -v sshpass &> /dev/null; then
        echo -e "${YELLOW}警告: 未找到 sshpass${NC}"
        echo -e "${YELLOW}建议安装 sshpass 以支持密码认证: sudo apt-get install sshpass${NC}"
        echo -e "${YELLOW}如果没有 sshpass，脚本将尝试使用其他方法，但可能不够稳定${NC}"
        echo ""
        read -p "是否继续？(y/n): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# 构建排除参数
build_exclude_args() {
    local exclude_patterns=(
        "build"
        "install"
        "log"
        ".git"
        "*.zip"
        "__pycache__"
        "*.pyc"
        "*.so"
        "*.a"
        "*.o"
        ".idea"
        ".vscode"
        "*.swp"
        "*.swo"
        "*~"
        ".DS_Store"
        ".gif"
        ".png"
    )
    
    local args=""
    for pattern in "${exclude_patterns[@]}"; do
        args="$args --exclude=$pattern"
    done
    echo "$args"
}

# 交互式密码输入
get_password() {
    local password
    read -sp "请输入远程服务器密码: " password
    echo ""
    echo "$password"
}

# 执行rsync同步
execute_rsync() {
    local src_path="$1"
    local dst_path="$2"
    local exclude_args="$3"
    local dry_run="$4"
    
    local rsync_opts="-avz --progress"
    
    if [ "$dry_run" = "true" ]; then
        rsync_opts="$rsync_opts --dry-run"
        echo -e "${YELLOW}预览模式：不会实际同步文件${NC}"
    fi
    
    # 检查是否安装了sshpass
    if command -v sshpass &> /dev/null; then
        # 使用sshpass（推荐方式）
        local password=$(get_password)
        SSHPASS="$password" sshpass -e rsync $rsync_opts $exclude_args "$src_path" "$dst_path"
    else
        # 如果没有sshpass，提示用户手动输入密码
        # 注意：rsync通过SSH连接，会提示输入密码
        echo -e "${YELLOW}注意: 由于未安装 sshpass，rsync 可能会多次提示输入密码${NC}"
        echo -e "${YELLOW}建议安装 sshpass: sudo apt-get install sshpass${NC}"
        echo ""
        
        # 使用expect作为备选方案（如果可用）
        if command -v expect &> /dev/null; then
            local password=$(get_password)
            expect << EOF
spawn rsync $rsync_opts $exclude_args "$src_path" "$dst_path"
expect {
    "password:" {
        send "$password\r"
        exp_continue
    }
    "yes/no" {
        send "yes\r"
        exp_continue
    }
    eof
}
wait
EOF
        else
            # 最后的选择：直接运行rsync，让用户手动输入密码
            echo -e "${YELLOW}将使用标准SSH密码提示，请手动输入密码${NC}"
            rsync $rsync_opts $exclude_args "$src_path" "$dst_path"
        fi
    fi
    
    return $?
}

# 同步函数
sync_code() {
    local direction=$1  # push 或 pull
    local dry_run=$2    # true 或 false
    local specific_pkg=$3  # 特定包名，可选
    
    # 检查远程主机是否设置
    if [ -z "$REMOTE_HOST" ]; then
        echo -e "${RED}错误: 未指定远程主机地址${NC}"
        echo -e "${YELLOW}请使用 -h 或 --host 参数指定远程IP或主机名${NC}"
        echo ""
        usage
        exit 1
    fi
    
    local exclude_args=$(build_exclude_args)
    
    if [ -n "$specific_pkg" ]; then
        # 只同步特定包
        local src_path="${LOCAL_PATH}/src/${specific_pkg}/"
        local dst_path="${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/src/${specific_pkg}/"
        
        if [ "$direction" = "pull" ]; then
            src_path="${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/src/${specific_pkg}/"
            dst_path="${LOCAL_PATH}/src/${specific_pkg}/"
        fi
        
        echo -e "${GREEN}同步包: ${specific_pkg}${NC}"
        echo -e "${BLUE}源: ${src_path}${NC}"
        echo -e "${BLUE}目标: ${dst_path}${NC}"
        echo ""
        
        execute_rsync "$src_path" "$dst_path" "$exclude_args" "$dry_run"
    else
        # 同步整个工作空间
        local src_path="${LOCAL_PATH}/"
        local dst_path="${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/"
        
        if [ "$direction" = "pull" ]; then
            src_path="${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH}/"
            dst_path="${LOCAL_PATH}/"
        fi
        
        echo -e "${GREEN}同步整个工作空间${NC}"
        echo -e "${BLUE}源: ${src_path}${NC}"
        echo -e "${BLUE}目标: ${dst_path}${NC}"
        echo ""
        
        # 对于完整同步，添加 --delete 选项（非dry-run模式）
        if [ "$dry_run" != "true" ]; then
            exclude_args="$exclude_args --delete"
        fi
        
        execute_rsync "$src_path" "$dst_path" "$exclude_args" "$dry_run"
    fi
    
    return $?
}

# 主逻辑
main() {
    # 先解析选项参数
    parse_args "$@"
    
    # 获取命令和包名（跳过已解析的选项）
    local command="push"
    local specific_pkg=""
    local prev_arg=""
    local skip_next=false
    
    for arg in "$@"; do
        # 如果上一个参数是选项，跳过当前参数（选项的值）
        if [ "$skip_next" = true ]; then
            skip_next=false
            prev_arg=""
            continue
        fi
        
        # 检查是否是选项
        case $arg in
            -h|--host|-u|--user|-p|--path)
                skip_next=true
                prev_arg="$arg"
                continue
                ;;
            --help)
                # 已经在parse_args中处理了
                continue
                ;;
            push|pull|dry-run|specific)
                command="$arg"
                prev_arg="$arg"
                ;;
            *)
                # 如果上一个参数是specific，当前参数就是包名
                if [ "$prev_arg" = "specific" ] && [ -z "$specific_pkg" ]; then
                    specific_pkg="$arg"
                fi
                prev_arg="$arg"
                ;;
        esac
    done
    
    # 检查依赖
    check_dependencies
    
    # 显示配置信息
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}代码同步工具${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}本地路径: ${LOCAL_PATH}${NC}"
    echo -e "${GREEN}远程用户: ${REMOTE_USER}${NC}"
    echo -e "${GREEN}远程主机: ${REMOTE_HOST}${NC}"
    echo -e "${GREEN}远程路径: ${REMOTE_PATH}${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # 执行同步
    case "$command" in
        push)
            echo -e "${GREEN}推送代码到远程服务器...${NC}"
            sync_code "push" "false" "$specific_pkg"
            ;;
        pull)
            echo -e "${GREEN}从远程服务器拉取代码...${NC}"
            sync_code "pull" "false" "$specific_pkg"
            ;;
        dry-run)
            echo -e "${YELLOW}预览同步（不实际执行）...${NC}"
            sync_code "push" "true" "$specific_pkg"
            ;;
        specific)
            if [ -z "$specific_pkg" ]; then
                echo -e "${RED}错误: 请指定要同步的包名${NC}"
                echo ""
                usage
                exit 1
            fi
            echo -e "${GREEN}同步包: ${specific_pkg}${NC}"
            sync_code "push" "false" "$specific_pkg"
            ;;
        *)
            echo -e "${RED}未知命令: $command${NC}"
            usage
            exit 1
            ;;
    esac
    
    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${GREEN}✅ 同步完成！${NC}"
    else
        echo ""
        echo -e "${RED}❌ 同步失败，请检查：${NC}"
        echo -e "${YELLOW}  - 网络连接是否正常${NC}"
        echo -e "${YELLOW}  - 密码是否正确${NC}"
        echo -e "${YELLOW}  - 远程路径是否存在${NC}"
        echo -e "${YELLOW}  - SSH服务是否运行${NC}"
        exit 1
    fi
}

# 运行主函数
main "$@"

