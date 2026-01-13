#!/usr/bin/bash

# Git Submodules Initialization Script
# Initialize all submodules based on tianracer_ros2_ws.repos file

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================================
# Submodule Configuration
# ============================================================================
# Uncomment the submodules you want to initialize
# Comment out the ones you want to skip
# ============================================================================

SUBMODULES=(
    # ["path"]="url|branch"
    ["src/ackermann_msgs"]="git@github.com:ros-drivers/ackermann_msgs.git|ros2"
    ["src/librealsense"]="git@github.com:IntelRealSense/librealsense.git|master"
    ["src/osight_lidar_ros2"]="git@github.com:TheWindISeek/osight_lidar_ros2.git|dev"
    ["src/realsense-ros"]="git@github.com:IntelRealSense/realsense-ros.git|ros2-development"
    ["src/slam_toolbox"]="git@github.com:SteveMacenski/slam_toolbox.git|master"
    ["src/tianbot_core_ros2"]="git@github.com:TheWindISeek/tianbot_core_ros2.git|dev"
)

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Git Submodules Initialization${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if we're in a git repository
if [ ! -d .git ]; then
    echo -e "${RED}Error: Current directory is not a git repository${NC}"
    echo "Please run: git init"
    exit 1
fi

# Check for embedded git repositories in configured submodule paths
found_embedded_repos=false
embedded_repos=()

for path in "${!SUBMODULES[@]}"; do
    if [ -d "$path/.git" ]; then
        found_embedded_repos=true
        embedded_repos+=("$path")
    fi
done

if [ "$found_embedded_repos" = true ]; then
    echo -e "${YELLOW}Found embedded git repositories in the following directories:${NC}"
    for repo in "${embedded_repos[@]}"; do
        echo -e "  - ${YELLOW}$repo${NC}"
    done
    echo ""
    echo -e "${YELLOW}These directories need to have their .git folders removed to be added as submodules.${NC}"
    read -p "Continue to remove these .git folders? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}Operation cancelled${NC}"
        exit 1
    fi
    
    echo -e "${YELLOW}Cleaning embedded git repositories...${NC}"
    for repo in "${embedded_repos[@]}"; do
        if [ -d "$repo/.git" ]; then
            rm -rf "$repo/.git"
            echo -e "  ${GREEN}Removed: $repo/.git${NC}"
        fi
    done
    echo ""
fi

# Check if .gitmodules already exists
if [ -f .gitmodules ]; then
    echo -e "${YELLOW}Detected existing .gitmodules file${NC}"
    read -p "Re-initialize submodules? This will remove existing submodule configuration (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Removing existing submodule configuration...${NC}"
        git submodule deinit -f --all 2>/dev/null || true
        # Remove all configured submodules from index
        for path in "${!SUBMODULES[@]}"; do
            git rm --cached -r "$path" 2>/dev/null || true
        done
        rm -f .gitmodules
        echo -e "${GREEN}Cleaned existing configuration${NC}"
        echo ""
    else
        echo -e "${YELLOW}Operation cancelled. To update existing submodules, run:${NC}"
        echo "  git submodule update --init --recursive"
        exit 0
    fi
fi

# Remove directories from index if they exist (if not already submodules)
echo -e "${YELLOW}Checking and cleaning directories in index...${NC}"
for path in "${!SUBMODULES[@]}"; do
    if git ls-files --error-unmatch "$path" >/dev/null 2>&1; then
        echo -e "  Removing from index: ${YELLOW}$path${NC}"
        git rm --cached -r "$path" 2>/dev/null || true
    fi
done
echo ""

# Check and clean existing directories and .git/modules cache
echo -e "${YELLOW}Checking and cleaning existing directories and submodule cache...${NC}"
need_cleanup=false
dirs_to_remove=()

for path in "${!SUBMODULES[@]}"; do
    if [ -d "$path" ]; then
        need_cleanup=true
        dirs_to_remove+=("$path")
    fi
    # Clean .git/modules cache
    if [ -d ".git/modules/$path" ]; then
        need_cleanup=true
        echo -e "  Found submodule cache: ${YELLOW}.git/modules/$path${NC}"
    fi
done

if [ "$need_cleanup" = true ]; then
    echo -e "${YELLOW}Detected the following directories or cache that need cleanup:${NC}"
    for dir in "${dirs_to_remove[@]}"; do
        echo -e "  - ${YELLOW}$dir${NC}"
    done
    echo ""
    echo -e "${YELLOW}These directories need to be deleted to be added as submodules.${NC}"
    read -p "Continue to delete these directories? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}Operation cancelled${NC}"
        exit 1
    fi
    
    echo -e "${YELLOW}Cleaning directories and cache...${NC}"
    for dir in "${dirs_to_remove[@]}"; do
        if [ -d "$dir" ]; then
            rm -rf "$dir"
            echo -e "  ${GREEN}Deleted: $dir${NC}"
        fi
    done
    # Clean .git/modules cache
    for path in "${!SUBMODULES[@]}"; do
        if [ -d ".git/modules/$path" ]; then
            rm -rf ".git/modules/$path"
            echo -e "  ${GREEN}Cleaned cache: .git/modules/$path${NC}"
        fi
    done
    echo ""
fi

# Add submodules
echo -e "${GREEN}Adding Git submodules...${NC}"
echo ""

success_count=0
fail_count=0

for path in "${!SUBMODULES[@]}"; do
    # Parse URL and branch from the value (format: "url|branch")
    value="${SUBMODULES[$path]}"
    url="${value%%|*}"
    branch="${value#*|}"
    module_name=$(basename "$path")
    
    echo -e "${BLUE}Adding $module_name...${NC}"
    if git submodule add -b "$branch" "$url" "$path" 2>&1; then
        echo -e "  ${GREEN}✓ Successfully added${NC}"
        success_count=$((success_count + 1))
    else
        error_msg=$(git submodule add -b "$branch" "$url" "$path" 2>&1)
        echo -e "  ${RED}✗ Failed to add: $error_msg${NC}"
        fail_count=$((fail_count + 1))
    fi
done

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Submodule initialization completed!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${GREEN}Successfully added: ${success_count} submodule(s)${NC}"
if [ $fail_count -gt 0 ]; then
    echo -e "${RED}Failed to add: ${fail_count} submodule(s)${NC}"
fi
echo ""
echo -e "${YELLOW}Notes:${NC}"
echo "1. Submodules have been added to .gitmodules file"
echo "2. To initialize all submodules, run: git submodule update --init --recursive"
echo "3. To update all submodules, run: git submodule update --remote"
echo "4. To clone repository with submodules, run: git clone --recursive <repo-url>"
echo ""

