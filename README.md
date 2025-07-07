# POC ROS 2 – FCL + KDL

This package (`fcl_coll`) demonstrates how to perform:

- ✅ **Collision detection** using the [Flexible Collision Library (FCL)](https://github.com/flexible-collision-library/fcl)
- ✅ **Forward and Inverse Kinematics** using [KDL](https://wiki.ros.org/orocos_kdl)  
- ✅ **URDF loading**, **TF broadcasting**, and **RViz2 visualization**

The system can detect self-collisions and environment collisions, solve IK for goal poses, and visualize both the robot and collisions.

---

## 🔁 Clone the Repository

```bash
git clone git@github.com:vanshgupta4452/poc-ros2.git
```

---

## 📦 Install Dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-rclcpp \
  ros-humble-urdf \
  ros-humble-geometric-shapes \
  ros-humble-tf2-ros \
  ros-humble-resource-retriever \
  ros-humble-tf2-geometry-msgs \
  libfcl-dev \
  libeigen3-dev \
  assimp-utils \
  libassimp-dev \
  ros-humble-kdl-parser \
  ros-humble-orocos-kdl
```

---

## ⚙️ Build the Package

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash
```

---

## ▶ Run the Nodes

```bash
# Run FCL-based environment collision checker
ros2 launch PXA-100_description display.launch.py

# Run self-collision check node
ros2 run kdl_ik_solver kdl_ik_solver_node


```
