# Robocar Webots Simulator

A ROS2-based robotic car simulator using Webots for training autonomous driving AI. Features smooth keyboard control and external C++ controller architecture.

## 🚗 Features

- **Webots Integration**: Full physics simulation with 4-wheel differential drive
- **ROS2 Architecture**: Communication via topics (`/cmd_vel`)
- **External C++ Controller**: Non-blocking FIFO-based controller for real-time motor control
- **Smooth Keyboard Control**: Progressive acceleration/deceleration for realistic movement
- **Modular Design**: Separated keyboard input, ROS2 bridge, and Webots controller

## 📋 Prerequisites

- **Webots R2025a** (or compatible version)
- **ROS2 Jazzy** (or compatible distribution)
- **Ubuntu 22.04+** (or compatible Linux distribution)
- **C++ compiler** (g++)
- **Python 3.12+**

## 🗂️ Project Structure

```
robocar_webots_sim/
├── controllers/
│   └── robocar_extern_controller/
│       ├── robocar_extern_controller.cpp    # C++ external controller
│       └── Makefile                         # Build configuration
├── protos/
│   └── Robocar.proto                        # Robot PROTO definition
├── worlds/
│   └── training_world.wbt                   # Webots world file
├── src/robocar_driver/
│   ├── keyboard_controller.py               # Keyboard input node
│   └── webots_bridge.py                     # ROS2 to FIFO bridge
├── launch_simulation.sh                     # Main launch script
└── README.md
```

## 🔧 Installation

### 1. Install Dependencies

```bash
# Install ROS2 Jazzy (if not already installed)
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install Webots (if not already installed)
# Download from: https://cyberbotics.com/
```

### 2. Build the Project

```bash
cd ~/Delivery/robocar/robocar_webots_sim

# Build ROS2 packages
colcon build

# Build C++ controller
cd controllers/robocar_extern_controller
export WEBOTS_HOME=/usr/local/webots
make clean
make
```

## 🚀 Usage

### Quick Start (Automated Launch)

```bash
cd ~/Delivery/robocar/robocar_webots_sim
./launch_simulation.sh
```

The script will:
1. Launch Webots with the training world
2. Start the C++ external controller
3. Start the ROS2 bridge
4. Display instructions for keyboard control

### Manual Launch

**Terminal 1 - Webots:**
```bash
/usr/local/webots/webots worlds/training_world.wbt
```

**Terminal 2 - C++ Controller:**
```bash
export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH
./controllers/robocar_extern_controller/robocar_extern_controller < /tmp/robocar_input_fifo
```

**Terminal 3 - ROS2 Bridge:**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robocar_driver webots_bridge --ros-args -p fifo_path:=/tmp/robocar_input_fifo
```

**Terminal 4 - Keyboard Controller:**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robocar_driver keyboard_controller
```

## 🎮 Controls

| Key | Action |
|-----|--------|
| **W** | Forward (increase speed) |
| **S** | Backward (decrease speed) |
| **A** | Turn left (hold to maintain) |
| **D** | Turn right (hold to maintain) |
| **Q** | Decrease max speed |
| **E** | Increase max speed |
| **SPACE** | Emergency stop |
| **ESC** | Exit |

## ⚙️ Configuration

### Adjust Rotation Sensitivity

Edit `controllers/robocar_extern_controller/robocar_extern_controller.cpp`:

```cpp
float angularFactor = 1.2;  // Higher = sharper turns (0.5 - 2.0)
float maxSpeed = 20.0;      // Max wheel velocity (rad/s)
```

### Adjust Movement Smoothness

Edit `src/robocar_driver/keyboard_controller.py`:

```python
self.linear_acceleration = 0.2   # Lower = smoother (0.1 - 0.5)
self.angular_acceleration = 0.25
self.max_turn_speed = 5.0        # Max angular velocity
```

After changes, rebuild:
```bash
cd controllers/robocar_extern_controller && make
cd ~/Delivery/robocar/robocar_webots_sim && colcon build
```

## 🐛 Troubleshooting

### Robot doesn't move

**Test FIFO directly:**
```bash
echo "5.0,0.0" > /tmp/robocar_input_fifo
```
- If robot moves: Issue is in ROS2 bridge or keyboard controller
- If robot doesn't move: Issue is in C++ controller

**Check logs:**
```bash
tail -f /tmp/robocar_controller.log
tail -f /tmp/robocar_bridge.log
```

### "libCppController.so not found" error

```bash
export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH
```

### ROS2 topics not working

```bash
# Verify topics exist
ros2 topic list

# Check if messages are published
ros2 topic echo /cmd_vel

# Verify nodes are running
ros2 node list
```

## 📊 Architecture

```
Keyboard Input → ROS2 Node → /cmd_vel Topic → Bridge Node → FIFO → C++ Controller → Webots Motors
```

**Communication Flow:**
1. User presses keys (keyboard_controller.py)
2. Twist messages published to `/cmd_vel` topic
3. Bridge node (webots_bridge.py) receives messages
4. Bridge writes `linear,angular` to FIFO pipe
5. C++ controller reads from FIFO (non-blocking)
6. Controller computes differential wheel speeds
7. Motor velocities applied in Webots simulation

## 🔬 Technical Details

### Robot Specifications
- **Type**: 4-wheel differential drive
- **Wheel radius**: 0.05m
- **Wheel separation**: 0.3m (left-right)
- **Mass**: 8kg (chassis) + 0.5kg per wheel
- **Max velocity**: 20 rad/s per motor

### Differential Drive Kinematics
```cpp
leftSpeed = linear + (angular * angularFactor)
rightSpeed = linear - (angular * angularFactor)
```

## 📝 License

MIT License

## 👥 Contributors

Developed for autonomous driving AI training with Webots simulator.

## 📧 Support

For issues or questions, check the troubleshooting section or review the log files.
