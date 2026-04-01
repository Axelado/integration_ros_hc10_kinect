# 🤖 Robotic Integration — 3D Object Scanning with Yaskawa HC10

## 📋 Description

This project implements an autonomous 3D object scanning system using a **Yaskawa HC10** robotic arm equipped with a **Kinect 3D sensor**. The robot generates an elliptical trajectory around an unknown object to digitize it completely, while avoiding obstacles in real time.

Developed as part of the **SRI (Systèmes Robotiques et Interactifs)** curriculum at **UPSSITECH — Université Paul Sabatier (Toulouse III)**.

---

## 🛠️ Tech Stack

| Tool | Role |
|---|---|
| **ROS Noetic** | Core robotics middleware |
| **Gazebo** | Physics simulation environment |
| **MoveIt** | Motion planning & trajectory execution |
| **ROS Control** | Low-level robot controller |
| **OctoMap** | 3D occupancy map generation |
| **Python 3** | Scripting & MoveIt API |
| **Visual Studio Code** | Development IDE |

> Platform: **Ubuntu 20.04** (Virtual Machine)

---

## 🎯 Project Goals

1. Simulate the Yaskawa HC10 arm with an attached Kinect sensor in **Gazebo**
2. Acquire and filter **3D point cloud data** from the Kinect
3. Generate an **adaptive elliptical trajectory** around the target object
4. Execute the trajectory via the **MoveIt Python API** with obstacle avoidance

---

## 📦 Packages

| Package | Description |
|---|---|
| `hc10_kinect_bringup` | System integration and startup |
| `hc10_kinect_description` | Robot URDF/Xacro definitions (HC10 + Kinect) |
| `hc10_kinect_gazebo` | Gazebo simulation setup |
| `hc10_kinect_capteur` | Kinect data processing and point cloud filtering |
| `hc10_kinect_moveit_config` | MoveIt configuration (planners, controllers, kinematics) |
| `hc10_kinect_ikfast_plugin` | IKFast inverse kinematics solver plugin |
| `hc10_kinect_object_scanning` | Elliptical trajectory generation and execution |

---

## 🏗️ Architecture

The project is divided into 3 main modules:

### 1. 🖥️ Simulation

- URDF/Xacro robot description with Kinect integration (`hc10_kinect_description`)
- Kinect modeled as an end-effector attached to `tool0` with an optical axis link (`kinect_link_optical`)
- Gazebo depth camera plugin: **640×480** resolution, **60° FOV**, **20 Hz** update rate
- IKFast solver used for inverse kinematics

### 2. 📡 Sensor & Data Processing

**Key ROS topics:**

| Topic | Content |
|---|---|
| `/camera/depth/image_raw` | Raw depth images |
| `/camera/color/image_raw` | RGB images |
| `/camera/depth/points` | 3D point cloud *(primary input)* |
| `/camera/depth/points_black` | Filtered point cloud *(output)* |

**Filtering pipeline** (`second_iteration.py`):
- Point clustering using normal estimation relative to the point cloud centroid
- Ground plane detection and removal (threshold: `s = 0.196 × |max_z − min_z|`)
- Outlier filtering by Z-axis height threshold
- Filtered cloud published on `/camera/depth/points_black`

**OctoMap integration** (`kinect_octomap.launch`):
- `octomap_server_node` converts filtered point clouds into a 3D occupancy map
- Used by MoveIt for collision-aware planning (max range: **5.0 m**, padding: **0.1 m**)

### 3. 🛤️ Trajectory Generation

**Ellipse computation** (`ellipse.py`):
- Bounding box of the object point cloud used to compute ellipse center and semi-axes
- Ellipse scaled by a safety coefficient to guarantee collision-free clearance
- Formula: `x = C.x + a·cos(t)·scale`, `y = C.y + b·sin(t)·scale`
- `scale = 1.38`, `ε = 0.12`
- 12 waypoints generated per revolution

**Multi-layer scanning:**
- Number of revolutions determined by object height (1 layer per 10 units of height)

**Camera orientation:**
- Quaternion computed at each waypoint to keep the camera pointed at the object center
- Uses rotation axis `δ = Z × d` and angle `θ = arccos(Z · d)`

**MoveIt control:**
- `moveit_commander` Python library used to send Cartesian poses
- Each waypoint (x, y, z + quaternion) sent sequentially to the MoveIt API

---

## 🚀 Getting Started

### Prerequisites

- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on Ubuntu 20.04
- Gazebo (included with `ros-noetic-desktop-full`)
- MoveIt: `sudo apt install ros-noetic-moveit`
- OctoMap: `sudo apt install ros-noetic-octomap ros-noetic-octomap-server`
- Python dependencies: `rospy`, `moveit_commander`, `sensor_msgs`, `numpy`

### Build

```bash
# Clone into your catkin workspace
cd ~/catkin_ws/src
git clone <repository_url> integration_ros_hc10_kinect

# Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Launch

**1. Start the full simulation (Gazebo + MoveIt + OctoMap):**

```bash
roslaunch hc10_kinect_moveit_config demo_gazebo.launch
```

**2. Start the point cloud filter:**

```bash
rosrun hc10_kinect_capteur second_iteration.py
```

**3. Run the scanning trajectory:**

```bash
rosrun hc10_kinect_object_scanning ellipse.py
```

**Alternative — MoveIt demo only (no Gazebo, fake controllers):**

```bash
roslaunch hc10_kinect_moveit_config demo.launch
```

---

## 📊 Results

Tested on multiple objects including the **Kuka YouBot** robot model:

| Metric | Value |
|---|---|
| 3D digitization | ✅ Complete via OctoMap in RViz |
| Max position error | **~5×10⁻² mm** |
| Max orientation error | **~8×10⁻² rad** |

---

## 📚 References

- [MoveIt Setup Assistant](https://moveit.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html)
- [Gazebo Simulation Integration](https://moveit.github.io/moveit_tutorials/doc/gazebo_simulation/gazebo_simulation.html)
- [Move Group Python Interface](https://moveit.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)
- [Mesh Filter with UR5 and Kinect](https://moveit.github.io/moveit_tutorials/doc/mesh_filter/mesh_filter_tutorial.html)
- [Perception Pipeline Tutorial](https://moveit.github.io/moveit_tutorials/doc/perception_pipeline/perception_pipeline_tutorial.html)
