#!/usr/bin/env bash
set -e

CONTAINER_NAME="visionflow-px4-sitl"
WORKDIR="/workspace/VisionFlow-PX4"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    echo "[ERROR] Container is not running: ${CONTAINER_NAME}"
    echo ""
    echo "Please start PX4/Gazebo SITL first:"
    echo "  bash docker/run_gz_sitl.sh"
    echo ""
    echo "Current running containers:"
    docker ps --format '  {{.Names}}\t{{.Status}}'
    exit 1
fi

docker exec -it \
    -e TERM=xterm-256color \
    "${CONTAINER_NAME}" \
    bash -lc "
        cat > /tmp/visionflow_interactive_bashrc <<'EOF'
# ===== VisionFlow PX4/Gazebo interactive shell =====

export TERM=xterm-256color
export FORCE_COLOR=1
export CLICOLOR=1
export LS_COLORS='di=01;34:ln=01;36:so=01;35:pi=33:ex=01;32:bd=33;01:cd=33;01:su=37;41:sg=30;43:tw=30;42:ow=34;42'

# ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Project environment, if built later
if [ -f /workspace/VisionFlow-PX4/install/setup.bash ]; then
    source /workspace/VisionFlow-PX4/install/setup.bash
fi

# Useful aliases
alias ls='ls --color=auto'
alias ll='ls -alF --color=auto'
alias la='ls -A --color=auto'
alias l='ls -CF --color=auto'
alias grep='grep --color=auto'
alias egrep='egrep --color=auto'
alias fgrep='fgrep --color=auto'

alias croot='cd /workspace/VisionFlow-PX4'
alias gzlist='gz topic -l'
alias gzps='ps aux | grep -E \"gz|px4|ros|bridge\" | grep -v grep'

parse_git_branch() {
    git branch 2>/dev/null | sed -n '/\\* /s///p'
}

show_git_branch() {
    local branch
    branch=\$(parse_git_branch)
    if [ -n \"\$branch\" ]; then
        echo \" (\$branch)\"
    fi
}

# Colored prompt:
# green user@host : blue path : yellow git branch
PS1='\\[\\033[01;32m\\]\\u@\\h\\[\\033[00m\\]:\\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\[\\033[01;33m\\]\$(show_git_branch)\\[\\033[00m\\]\\$ '

cd /workspace/VisionFlow-PX4 2>/dev/null || cd ~

echo \"\"
echo \"[OK] Entered PX4/Gazebo container: visionflow-px4-sitl\"
echo \"[OK] ROS_DISTRO=\${ROS_DISTRO:-unknown}\"
echo \"[OK] Workdir: \$(pwd)\"
echo \"\"
echo \"Useful commands:\"
echo \"  gz topic -l\"
echo \"  ros2 topic list\"
echo \"  gz topic -t /joint/gamma/1/position_cmd -m gz.msgs.Double -p 'data: 0.5'\"
echo \"\"
EOF

        exec bash --rcfile /tmp/visionflow_interactive_bashrc -i
    "
