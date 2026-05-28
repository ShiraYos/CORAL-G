#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# CORAL-G Lab Setup Script
# Usage: bash ~/CORAL-G/lab_setup.sh <ROS_DOMAIN_ID>
# ─────────────────────────────────────────────────────────────────────────────

CORAL_G_WS="$HOME/CORAL-G"

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
BLU='\033[0;34m'
CYN='\033[0;36m'
BLD='\033[1m'
NC='\033[0m'

HDR="${BLD}${CYN}"   # header colour
CMD="${CYN}"          # command colour
OPT="${YLW}"          # option letter colour
WRN="${YLW}"          # warning colour
ERR="${RED}"          # error colour
OK="${GRN}"           # success colour

# ── Helpers ───────────────────────────────────────────────────────────────────
divider() { echo -e "${HDR}═══════════════════════════════════════════════════════════════${NC}"; }
header()  { echo -e "${HDR}╔══════════════════════════════════════════╗${NC}"
            echo -e "${HDR}║  $1${NC}"
            echo -e "${HDR}╚══════════════════════════════════════════╝${NC}"; }

# ── [1] Argument check ────────────────────────────────────────────────────────
if [ -z "$1" ]; then
    echo -e "${ERR}ERROR: ROS_DOMAIN_ID is required.${NC}"
    echo ""
    echo "  Usage:   bash ~/CORAL-G/lab_setup.sh <ROS_DOMAIN_ID>"
    echo "  Example: bash ~/CORAL-G/lab_setup.sh 33"
    echo ""
    echo "  ROS_DOMAIN_ID is assigned at the start of each lab session."
    exit 1
fi

export ROS_DOMAIN_ID="$1"

echo ""
header "CORAL-G Lab Setup                        "
echo -e "  ROS_DOMAIN_ID  = ${YLW}${ROS_DOMAIN_ID}${NC}"
echo -e "  Workspace      = ${YLW}${CORAL_G_WS}${NC}"
echo ""

# ── [2] Source ROS 2 Jazzy ────────────────────────────────────────────────────
echo -e "${BLU}[1/4]${NC} Sourcing ROS 2 Jazzy..."
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo -e "  ${ERR}✗ /opt/ros/jazzy/setup.bash not found. Is ROS 2 Jazzy installed?${NC}"
    exit 1
fi
source /opt/ros/jazzy/setup.bash
echo -e "  ${OK}✓${NC} /opt/ros/jazzy/setup.bash"

# ── [3] Source TurtleBot3 workspace ──────────────────────────────────────────
echo -e "${BLU}[2/4]${NC} Sourcing TurtleBot3 workspace..."
if [ ! -f "$HOME/turtlebot3_ws/install/setup.bash" ]; then
    echo -e "  ${ERR}✗ ~/turtlebot3_ws/install/setup.bash not found.${NC}"
    echo -e "  ${ERR}  TurtleBot3 dependencies must be built separately.${NC}"
    exit 1
fi
source "$HOME/turtlebot3_ws/install/setup.bash"
echo -e "  ${OK}✓${NC} ~/turtlebot3_ws/install/setup.bash"

# ── [4] Set environment ───────────────────────────────────────────────────────
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
echo -e "  ${OK}✓${NC} TURTLEBOT3_MODEL=burger  LDS_MODEL=LDS-02  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# ── [5] Check for stale build artifacts ──────────────────────────────────────
echo -e "${BLU}[3/4]${NC} Checking CORAL-G workspace..."

if [ ! -d "$CORAL_G_WS" ]; then
    echo -e "  ${ERR}✗ $CORAL_G_WS not found. Clone the repo first:${NC}"
    echo -e "  ${CMD}  git clone https://github.com/ShiraYos/CORAL-G.git ~/CORAL-G${NC}"
    exit 1
fi

NEED_BUILD=false

if [ ! -f "$CORAL_G_WS/install/setup.bash" ]; then
    echo -e "  ${WRN}install/setup.bash not found — will build.${NC}"
    NEED_BUILD=true
elif ! grep -q "$CORAL_G_WS" "$CORAL_G_WS/install/setup.bash" 2>/dev/null; then
    echo -e "  ${WRN}Build artifacts are from a different machine. Cleaning and rebuilding...${NC}"
    rm -rf "$CORAL_G_WS/build" "$CORAL_G_WS/install" "$CORAL_G_WS/log"
    NEED_BUILD=true
else
    echo -e "  ${OK}✓${NC} install/setup.bash exists and matches this machine"
fi

if [ "$NEED_BUILD" = true ]; then
    cd "$CORAL_G_WS"
    echo -e "  Building my_tb3_world..."
    if ! colcon build --packages-select my_tb3_world 2>&1; then
        echo ""
        echo -e "  ${ERR}✗ Build failed. Fix the errors above before proceeding.${NC}"
        exit 1
    fi
    echo -e "  ${OK}✓${NC} Build complete"
fi

source "$CORAL_G_WS/install/setup.bash"
echo -e "  ${OK}✓${NC} ~/CORAL-G/install/setup.bash sourced"

# ── [6] Verify robot connectivity ────────────────────────────────────────────
echo -e "${BLU}[4/4]${NC} Checking robot topics (5 s timeout each)..."

check_topic() {
    local topic="$1"
    if timeout 5 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
        echo -e "  ${OK}✓${NC} $topic is publishing"
    else
        echo -e "  ${WRN}✗${NC} $topic not detected  — robot may not be connected yet"
    fi
}

check_topic /scan
check_topic /odom

# ── Shorthand used in every printed command ───────────────────────────────────
SRC="source ~/CORAL-G/source_coral.sh ${ROS_DOMAIN_ID}"
GOAL_PUB="ros2 topic pub --keep-alive 5 --once /coral_g/goal_pose geometry_msgs/msg/PoseStamped"
POSE="'{header: {frame_id: \"map\"}, pose: {position: {x: %s, y: %s, z: 0.0}, orientation: {w: 1.0}}}'"

goal_cmd() {
    # goal_cmd X Y
    printf "${CMD}${SRC} && ${GOAL_PUB} '${POSE}'${NC}\n" "$1" "$2" | sed "s/%s/$1/;s/%s/$2/"
    echo -e "     ${CMD}${SRC} && ${GOAL_PUB} '{header: {frame_id: \"map\"}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {w: 1.0}}}'${NC}"
}

# ── Menu ──────────────────────────────────────────────────────────────────────
echo ""
divider
echo -e "${BLD}  CORAL-G Command Menu  —  paste into a fresh terminal${NC}"
divider

# ─ PART 1 ─
echo ""
echo -e "${BLD}  PART 1 — Physical Robot Only${NC}"
echo ""

echo -e "${OPT}  A)${NC} SSH into robot"
echo -e "     ${CMD}ssh ubuntu@<ROBOT_IP>${NC}   ${WRN}← replace <ROBOT_IP> with the robot's IP address${NC}"
echo ""

echo -e "${OPT}  B)${NC} Verify robot visible  (/scan and /odom)"
echo -e "     ${CMD}${SRC} && ros2 topic hz /scan${NC}"
echo -e "     ${CMD}${SRC} && ros2 topic hz /odom${NC}"
echo ""

echo -e "${OPT}  C)${NC} Obstacle avoidance node  ${WRN}← do NOT run alongside Nav2${NC}"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world objectAvoidance_node.launch.py${NC}"
echo ""

echo -e "${OPT}  D)${NC} State sync node"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world stateSync_node.launch.py${NC}"
echo ""

echo -e "${OPT}  E)${NC} Twin input node"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world twin_input_node.launch.py${NC}"
echo ""

echo -e "${OPT}  F)${NC} Teleop keyboard"
echo -e "     ${CMD}${SRC} && ros2 run teleop_twist_keyboard teleop_twist_keyboard${NC}"
echo ""

divider

# ─ PART 2 ─
echo ""
echo -e "${BLD}  PART 2 — Physical Robot + Gazebo Digital Twin${NC}"
echo -e "  ${WRN}Launch order matters: G → H → I → J → K — wait for each step to fully start.${NC}"
echo ""

echo -e "${OPT}  G)${NC} Launch CORAL-G Gazebo world  ${WRN}← start here${NC}"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world new_world.launch.py${NC}"
echo ""

echo -e "${OPT}  H)${NC} Launch Gazebo twin  ${WRN}← only after G is fully loaded${NC}"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world gazebo_twin.launch.py${NC}"
echo ""

echo -e "${OPT}  I)${NC} Launch Nav2 + SLAM  ${WRN}← wait for 'Navigation status: ready' before sending goals${NC}"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world nav2_slam_navigation.launch.py use_sim_time:=false${NC}"
echo ""

echo -e "${OPT}  J)${NC} State sync node"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world stateSync_node.launch.py${NC}"
echo ""

echo -e "${OPT}  K)${NC} Twin input node"
echo -e "     ${CMD}${SRC} && ros2 launch my_tb3_world twin_input_node.launch.py${NC}"
echo ""

echo -e "${OPT}  L)${NC} Watch navigation status"
echo -e "     ${CMD}${SRC} && ros2 topic echo /coral_g/navigation_status${NC}"
echo ""

echo -e "${OPT}  M)${NC} Send test goal  →  (0.5, 0.0)"
echo -e "     ${CMD}${SRC} && ${GOAL_PUB} '{header: {frame_id: \"map\"}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'${NC}"
echo ""

echo -e "${OPT}  N)${NC} Send goal cluster 1  →  (1.5, 1.8)"
echo -e "     ${CMD}${SRC} && ${GOAL_PUB} '{header: {frame_id: \"map\"}, pose: {position: {x: 1.5, y: 1.8, z: 0.0}, orientation: {w: 1.0}}}'${NC}"
echo ""

echo -e "${OPT}  O)${NC} Send goal cluster 2  →  (-1.5, -1.5)"
echo -e "     ${CMD}${SRC} && ${GOAL_PUB} '{header: {frame_id: \"map\"}, pose: {position: {x: -1.5, y: -1.5, z: 0.0}, orientation: {w: 1.0}}}'${NC}"
echo ""

echo -e "${OPT}  P)${NC} Return to origin  →  (0.0, 0.0)"
echo -e "     ${CMD}${SRC} && ${GOAL_PUB} '{header: {frame_id: \"map\"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'${NC}"
echo ""

echo -e "${OPT}  Q)${NC} Verify all coral_g topics"
echo -e "     ${CMD}${SRC} && ros2 topic list | grep coral_g${NC}"
echo ""

echo -e "${OPT}  R)${NC} Read robot state once"
echo -e "     ${CMD}${SRC} && ros2 topic echo /coral_g/robot_state --once${NC}"
echo ""

divider

# ─ UTILITIES ─
echo ""
echo -e "${BLD}  UTILITIES${NC}"
echo ""

echo -e "${OPT}  S)${NC} ${RED}Emergency stop${NC}"
echo -e "     ${CMD}${SRC} && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{header: {frame_id: \"base_link\"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'${NC}"
echo ""

echo -e "${OPT}  T)${NC} Rebuild package  (deletes build / install / log and rebuilds)"
echo -e "     ${CMD}cd ~/CORAL-G && rm -rf build install log && source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash && colcon build --packages-select my_tb3_world && source install/setup.bash${NC}"
echo ""

divider
echo ""
echo -e "${OK}Environment ready.${NC} Dropping into sourced shell — ${BLD}Ctrl+D${NC} to exit."
echo -e "Prompt shows ${BLD}(CORAL-G|D=${ROS_DOMAIN_ID})${NC} to confirm your domain ID."
echo ""

# ── Drop into sourced interactive shell ───────────────────────────────────────
exec bash --rcfile <(cat <<RCEOF
[ -f ~/.bashrc ] && source ~/.bashrc
source /opt/ros/jazzy/setup.bash
source $HOME/turtlebot3_ws/install/setup.bash
source $CORAL_G_WS/install/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
PS1="(CORAL-G|D=$ROS_DOMAIN_ID) \$ "
RCEOF
)
