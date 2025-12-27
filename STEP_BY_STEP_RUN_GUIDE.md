# Step-by-Step Run Guide: See What Each Part Does

This guide shows you how to run each component **one by one** so you can see what each part does and visualize the outputs.

---

## 🎯 Quick Overview: What Controls What

| Component | Controls/Does | Output Topics | What You'll See |
|-----------|---------------|---------------|-----------------|
| **robot_state_publisher** | Robot structure (URDF) | `/tf` | Coordinate frames |
| **RPLidar** | LiDAR sensor reading | `/scan` | Laser scan points |
| **Odometry** | Position tracking | `/odom` | Robot position (x, y, theta) |
| **EKF** | Sensor fusion | `/odom` (filtered), `/tf` | Smoothed position |
| **SLAM Toolbox** | Map building | `/map` | Occupancy grid map |
| **Motor Controller** | Motor movement | (subscribes to `/cmd_vel`) | Robot moves |
| **A* Planner** | Path planning | `/global_path` | Planned path line |
| **LWB Planner** | Local navigation | `/cmd_vel` | Velocity commands |

---

## 📋 Prerequisites

Before starting, make sure:

```bash
# 1. Source ROS2
source /opt/ros/jazzy/setup.bash

# 2. Build your workspace
cd ~/ros2_ws
colcon build

# 3. Source your workspace
source install/setup.bash

# 4. Check LiDAR is connected
ls -l /dev/ttyUSB0  # Should exist if LiDAR connected
```

---

## 🚀 Method 1: Run Everything Step-by-Step (Recommended for Learning)

Open **6 separate terminals**. We'll start each component one by one and show you how to see its output.

---

### **STEP 1: robot_state_publisher** (Foundation)

**What it does**: Publishes robot structure (links and joints) as TF transforms.

**Terminal 1**:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher /home/caterpillar/ros2_ws/urdf/your_robot.urdf
```

**What you'll see**: Node running, no errors.

**Visualize the output**:
```bash
# In a NEW terminal, check TF tree:
ros2 run tf2_tools view_frames
evince frames.pdf  # Opens PDF showing frame structure

# Check specific transform:
ros2 run tf2_ros tf2_echo base_link laser
# Output: Shows transform from base_link to laser (should be x=0.12, y=0, z=0.25)
```

**What this controls**: 
- ✅ Defines where `laser` frame is relative to `base_link`
- ✅ Other nodes use this to know LiDAR position

---

### **STEP 2: RPLidar Driver** (Sensor Input)

**What it does**: Reads LiDAR hardware and publishes laser scan data.

**Terminal 2** (wait 2 seconds after Terminal 1):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```

**What you'll see**: 
- "RPLidar running..."
- May show scan frequency

**Visualize the output** (in a NEW terminal):
```bash
# See scan data rate:
ros2 topic hz /scan
# Output: Shows frequency (e.g., "average rate: 10.0 Hz")

# See actual scan data:
ros2 topic echo /scan --once
# Output: Shows ranges array with distance measurements

# See scan info:
ros2 topic info /scan
# Output: Shows message type and publishers/subscribers
```

**What this controls**:
- ✅ Provides `/scan` topic with 360° distance measurements
- ✅ Used by SLAM and obstacle detection

**Visualize in RViz** (optional):
```bash
# In another terminal:
rviz2
# Then in RViz:
# 1. Add → By display type → LaserScan
# 2. Set Topic to: /scan
# 3. You'll see red dots showing obstacles around robot
```

---

### **STEP 3: Odometry Node** (Position Tracking)

**What it does**: Tracks robot position by integrating velocity commands.

**Terminal 3** (wait 3 seconds after Terminal 2):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rover_navigation odometry_node
```

**What you'll see**: 
- "Odometry Node Started"
- Shows wheel radius, wheel base, encoder resolution

**Visualize the output**:
```bash
# See odometry data:
ros2 topic echo /odom
# Output: Shows position (x, y) and orientation (quaternion)
# Initially: x=0.0, y=0.0, theta=0.0

# Monitor odometry rate:
ros2 topic hz /odom
# Output: Shows publishing frequency (should be ~20 Hz)
```

**What this controls**:
- ✅ Publishes `/odom` with robot's estimated position
- ✅ Position updates when robot moves (via /cmd_vel)

**Test it** (in another terminal):
```bash
# Send a velocity command to see odometry update:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Watch /odom - you'll see x position increase!
ros2 topic echo /odom
```

---

### **STEP 4: EKF (Extended Kalman Filter)** (Sensor Fusion)

**What it does**: Fuses multiple sensor sources for better position estimate.

**Terminal 4** (wait 2 seconds after Terminal 3):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_localization ekf_node --ros-args -p config_file:=/home/caterpillar/ros2_ws/config/ekf.yaml
```

**What you'll see**: 
- EKF node started
- May show sensor timeout warnings (normal if no IMU)

**Visualize the output**:
```bash
# Check EKF is publishing transforms:
ros2 run tf2_ros tf2_echo odom base_link
# Output: Shows transform from odom to base_link

# See filtered odometry:
ros2 topic echo /odom
# Output: Smoothed position data (if subscribed to /odom_raw)
```

**What this controls**:
- ✅ Publishes `odom → base_link` transform
- ✅ Filters noise from odometry
- ✅ Provides better pose estimate than raw odometry

---

### **STEP 5: SLAM Toolbox** (Map Building)

**What it does**: Builds a map of the environment using LiDAR scans.

**Terminal 5** (wait 3 seconds after Terminal 4):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p scan_topic:=/scan \
  -p map_frame:=map \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p config_file:=/home/caterpillar/ros2_ws/config/slam_params.yaml
```

**What you'll see**: 
- SLAM Toolbox started
- May take 10-30 seconds to initialize
- Shows scan matching info

**Visualize the output**:
```bash
# Check map topic exists:
ros2 topic list | grep map
# Should show: /map

# See map data (after initialization):
ros2 topic echo /map --once
# Output: Shows OccupancyGrid with data array
# - 0 = free space
# - 100 = occupied (obstacle)
# - -1 = unknown

# Check map publishing rate:
ros2 topic hz /map
# Output: Shows update frequency (usually 0.5 Hz)

# Check TF transform:
ros2 run tf2_ros tf2_echo map odom
# Output: Shows map → odom transform (updates as robot moves)
```

**What this controls**:
- ✅ Creates `/map` occupancy grid
- ✅ Publishes `map → odom` transform
- ✅ Tracks robot position in map frame

**Visualize in RViz** (best way to see the map):
```bash
# In another terminal:
rviz2

# In RViz:
# 1. Add → By display type → Map
# 2. Set Topic to: /map
# 3. Set Color Scheme: costmap
# 4. You'll see the map building in real-time!
#    - White = free space
#    - Black = obstacles
#    - Gray = unknown
```

---

### **STEP 6: RViz2** (Visualization - Optional but Recommended)

**What it does**: Visualizes all ROS2 data in 3D.

**Terminal 6**:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

**In RViz2, add these displays**:

1. **LaserScan** (to see LiDAR data):
   - Add → By display type → LaserScan
   - Topic: `/scan`
   - Size: 0.05
   - Color: Red (255, 0, 0)
   - **You'll see**: Red dots showing obstacles around robot

2. **Map** (to see SLAM map):
   - Add → By display type → Map
   - Topic: `/map`
   - Color Scheme: costmap
   - **You'll see**: Map building in real-time

3. **TF** (to see coordinate frames):
   - Add → By display type → TF
   - **You'll see**: Arrows showing coordinate frames (map, odom, base_link, laser)

4. **Odometry** (to see robot pose):
   - Add → By display type → Odometry
   - Topic: `/odom`
   - **You'll see**: Arrow showing robot position and orientation

---

## 🎮 Method 2: Run Everything at Once (Quick Start)

If you want to start everything quickly:

```bash
cd ~/ros2_ws
./start_slam_mapping.sh
```

This opens 6 terminals automatically. Then use the visualization commands above to see outputs.

---

## 🔍 How to See Detected Outputs: Complete Monitoring Guide

### **1. See LiDAR Detection (Laser Scans)**

```bash
# Real-time scan visualization:
ros2 topic echo /scan

# See scan statistics:
ros2 topic hz /scan
ros2 topic bw /scan  # Bandwidth

# Visualize in RViz (best):
rviz2
# Add LaserScan display → Topic: /scan
```

**What you'll see**: 
- Array of distance measurements (ranges)
- Each value = distance to obstacle in that direction
- 360° coverage around robot

---

### **2. See Odometry (Position Tracking)**

```bash
# Real-time position:
ros2 topic echo /odom

# See position updates:
watch -n 0.5 'ros2 topic echo /odom --once | grep -A 3 position'

# Visualize in RViz:
# Add Odometry display → Topic: /odom
```

**What you'll see**:
- `x, y, z`: Position coordinates
- `orientation`: Quaternion (robot heading)
- Updates as robot moves

---

### **3. See SLAM Map**

```bash
# Get map info:
ros2 topic echo /map --once | head -20

# Check map size:
ros2 topic echo /map --once | grep -E "width|height|resolution"

# Visualize in RViz (best):
# Add Map display → Topic: /map
```

**What you'll see**:
- Occupancy grid (2D array)
- White = free space
- Black = obstacles
- Gray = unknown areas
- Map grows as robot explores

---

### **4. See Obstacle Detection** (if obstacle_detection node is running)

```bash
# Start obstacle detection:
ros2 run rover_navigation obstacle_detection

# See detected obstacles:
ros2 topic echo /obstacles

# See occupancy grid:
ros2 topic echo /occupancy_grid --once
```

**What you'll see**:
- List of obstacle positions
- 2D grid showing obstacle locations

---

### **5. See Path Planning** (if navigation is running)

```bash
# Start navigation:
ros2 launch rover_navigation a_star_lwb_navigation.launch.py

# See global path:
ros2 topic echo /global_path

# See velocity commands:
ros2 topic echo /cmd_vel

# Visualize in RViz:
# Add Path display → Topic: /global_path
# Add MarkerArray display → Topic: /planner/debug/state
```

**What you'll see**:
- Path waypoints (list of poses)
- Velocity commands (linear.x, angular.z)
- Path line in RViz

---

### **6. See Motor Commands**

```bash
# See what motors receive:
ros2 topic echo /cmd_vel

# Monitor command rate:
ros2 topic hz /cmd_vel
```

**What you'll see**:
- `linear.x`: Forward/backward speed (m/s)
- `angular.z`: Rotation speed (rad/s)
- Commands sent to motor controller

---

## 📊 Complete Monitoring Dashboard

Create a monitoring script to see everything at once:

**Create file: `monitor_all.sh`**
```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== ROS2 Topic Monitor ==="
echo ""
echo "1. LiDAR Scan Rate:"
timeout 2 ros2 topic hz /scan 2>/dev/null || echo "  /scan not publishing"
echo ""
echo "2. Odometry Rate:"
timeout 2 ros2 topic hz /odom 2>/dev/null || echo "  /odom not publishing"
echo ""
echo "3. Map Updates:"
timeout 2 ros2 topic hz /map 2>/dev/null || echo "  /map not publishing"
echo ""
echo "4. Active Topics:"
ros2 topic list | grep -E "scan|odom|map|cmd_vel|path"
echo ""
echo "5. Active Nodes:"
ros2 node list
echo ""
echo "6. TF Frames:"
ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | head -5 || echo "  TF not ready"
```

Make it executable:
```bash
chmod +x monitor_all.sh
./monitor_all.sh
```

---

## 🧪 Testing Each Component

### **Test 1: Verify LiDAR is Working**

```bash
# Terminal 1: Start RPLidar
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0

# Terminal 2: Check scan data
ros2 topic echo /scan --once
# Should see: ranges array with numbers

# Terminal 3: Visualize
rviz2
# Add LaserScan → See red dots around robot
```

**Expected output**: Red dots in RViz showing obstacles around robot.

---

### **Test 2: Verify Odometry is Working**

```bash
# Terminal 1: Start odometry
ros2 run rover_navigation odometry_node

# Terminal 2: Send test command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Terminal 3: Watch odometry
ros2 topic echo /odom
# Should see: x position increasing
```

**Expected output**: Odometry x value increases when you send forward command.

---

### **Test 3: Verify SLAM is Building Map**

```bash
# Start full mapping stack:
./start_slam_mapping.sh

# Wait 30 seconds, then:
ros2 topic echo /map --once | grep -E "width|height|resolution"
# Should see: map dimensions

# In RViz: Add Map display
# Move robot around (manually or via teleop)
# Should see: Map building in real-time
```

**Expected output**: Map appears in RViz and grows as robot moves.

---

### **Test 4: Verify Navigation is Working**

```bash
# Terminal 1: Start navigation
ros2 launch rover_navigation a_star_lwb_navigation.launch.py

# Terminal 2: Send goal
ros2 topic pub -1 /goal geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}
}'

# Terminal 3: Watch path
ros2 topic echo /global_path

# Terminal 4: Watch commands
ros2 topic echo /cmd_vel
```

**Expected output**: 
- Path appears in RViz
- Velocity commands published
- Robot moves toward goal

---

## 🎯 Quick Reference: What to Check

| Want to See... | Command | Visualize In |
|----------------|---------|--------------|
| **LiDAR scans** | `ros2 topic echo /scan` | RViz: LaserScan |
| **Robot position** | `ros2 topic echo /odom` | RViz: Odometry |
| **Map** | `ros2 topic echo /map` | RViz: Map |
| **Path plan** | `ros2 topic echo /global_path` | RViz: Path |
| **Motor commands** | `ros2 topic echo /cmd_vel` | Terminal |
| **Obstacles** | `ros2 topic echo /obstacles` | RViz: Markers |
| **TF frames** | `ros2 run tf2_tools view_frames` | PDF viewer |

---

## 🐛 Troubleshooting: No Output?

### **No /scan topic?**
```bash
# Check LiDAR is connected:
ls -l /dev/ttyUSB0

# Check RPLidar node is running:
ros2 node list | grep rplidar

# Check for errors:
ros2 topic list  # Should see /scan
```

### **No /odom topic?**
```bash
# Check odometry node is running:
ros2 node list | grep odometry

# Check it's subscribed to /cmd_vel:
ros2 topic info /odom
```

### **No /map topic?**
```bash
# SLAM needs time to initialize (10-30 seconds)
# Check SLAM is running:
ros2 node list | grep slam

# Check it's receiving /scan:
ros2 topic info /map
```

### **Can't see in RViz?**
```bash
# Check Fixed Frame is set correctly:
# In RViz: Global Options → Fixed Frame → Set to "map" or "odom"

# Check displays are added:
# In RViz: Displays panel should show your displays

# Check topics exist:
ros2 topic list
```

---

## 📝 Summary: Step-by-Step Checklist

1. ✅ **robot_state_publisher** → Provides TF transforms
2. ✅ **RPLidar** → Provides `/scan` (laser data)
3. ✅ **Odometry** → Provides `/odom` (position)
4. ✅ **EKF** → Filters odometry, provides TF
5. ✅ **SLAM** → Provides `/map` (occupancy grid)
6. ✅ **RViz** → Visualizes everything

**To see outputs**:
- Use `ros2 topic echo <topic>` for raw data
- Use `rviz2` for visual visualization
- Use `ros2 topic hz <topic>` for rates

---

## 🎓 Next Steps

1. **Run mapping**: Start all components, move robot, watch map build
2. **Test navigation**: Send goals, watch path planning
3. **Monitor topics**: Use commands above to see what's happening
4. **Tune parameters**: Adjust speeds, thresholds in config files

**Happy exploring!** 🚀

