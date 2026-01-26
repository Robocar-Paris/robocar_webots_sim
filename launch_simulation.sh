#!/bin/bash

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE} Robocar Simulation - Full Launch${NC}"
echo -e "${BLUE}========================================${NC}"

PROJECT_DIR="$HOME/Delivery/robocar/robocar_webots_sim"
FIFO_PATH="/tmp/robocar_input_fifo"

export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH

# Clean old logs
rm -f /tmp/robocar_*.log
rm -f $FIFO_PATH

cd "$PROJECT_DIR"

# Create named pipe (FIFO)
mkfifo $FIFO_PATH 2>/dev/null
chmod 666 $FIFO_PATH

echo -e "${GREEN}✓ Named pipe created: $FIFO_PATH${NC}"

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Stopping all services...${NC}"
    kill $WEBOTS_PID 2>/dev/null
    kill $CONTROLLER_PID 2>/dev/null
    kill $BRIDGE_PID 2>/dev/null
    kill $KEYBOARD_PID 2>/dev/null
    rm -f $FIFO_PATH
    exit 0
}

trap cleanup SIGINT SIGTERM

# 1. Launch Webots
echo -e "${GREEN}[1/4]${NC} Launching ${YELLOW}Webots${NC}..."
/usr/local/webots/webots worlds/training_world.wbt > /tmp/robocar_webots.log 2>&1 &
WEBOTS_PID=$!

echo -e "${YELLOW}Wait for the robot to appear in Webots and the message `INFO: 'Robocar' extern controller: Waiting for local or remote connection on port 1234 targeting robot named 'Robocar'.` to appear, then press ENTER...${NC}"
read

# 2. Launch C++ controller with FIFO input
echo -e "${GREEN}[2/4]${NC} Launching ${YELLOW}C++ Controller${NC}..."
export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH

cat > $FIFO_PATH < /dev/null &

LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH \
./controllers/robocar_extern_controller/robocar_extern_controller < $FIFO_PATH \
> /tmp/robocar_controller.log 2>&1 &

CONTROLLER_PID=$!

sleep 2

echo -e "${YELLOW}Check logs: ${BLUE}tail -f /tmp/robocar_controller.log${NC}"
echo -e "${YELLOW}Wait to see 'Motors initialized!', then press ENTER...${NC}"

# 3. Launch ROS2 bridge
echo -e "${GREEN}[3/4]${NC} Launching ${YELLOW}Webots Bridge${NC}..."

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run robocar_driver webots_bridge --ros-args -p fifo_path:=$FIFO_PATH > /tmp/robocar_bridge.log 2>&1 &

BRIDGE_PID=$!

sleep 2

echo -e "${YELLOW}Check logs: ${BLUE}tail -f /tmp/robocar_bridge.log${NC}"
echo -e "${YELLOW}Wait to see 'Connected to FIFO', then press ENTER...${NC}"

# 4. Display instructions
echo -e "${GREEN}[4/4]${NC} Launching ${YELLOW}Keyboard Controller${NC}..."

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}✅ Simulation ready!${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "\n${YELLOW}📋 Available logs:${NC}"
echo -e " ${BLUE}tail -f /tmp/robocar_controller.log${NC} (Controller)"
echo -e " ${BLUE}tail -f /tmp/robocar_bridge.log${NC} (Bridge)"

echo -e "\n${YELLOW}🎮 Launch keyboard controller in a NEW TERMINAL:${NC}"
echo -e "${BLUE}cd ~/Delivery/robocar/robocar_webots_sim${NC}"
echo -e "${BLUE}source /opt/ros/jazzy/setup.bash && source install/setup.bash${NC}"
echo -e "${BLUE}ros2 run robocar_driver keyboard_controller${NC}"

echo -e "\n${YELLOW}Controls:${NC}"
echo -e " ${BLUE}W${NC} = Forward"
echo -e " ${BLUE}S${NC} = Backward"
echo -e " ${BLUE}A${NC} = Turn left"
echo -e " ${BLUE}D${NC} = Turn right"
echo -e " ${BLUE}Q/E${NC} = Decrease/Increase speed"
echo -e " ${BLUE}SPACE${NC} = Stop"
echo -e " ${BLUE}ESC${NC} = Exit"

echo -e "\n${YELLOW}📡 Bridge now writing to controller via: ${BLUE}$FIFO_PATH${NC}\n"

echo -e "${RED}Press CTRL+C here to STOP EVERYTHING${NC}\n"

# Wait for all processes
wait
