# FCL ROS 2

This package (`fcl_coll`) demonstrates how to perform collision detection using the [Flexible Collision Library (FCL)](https://github.com/flexible-collision-library/fcl) in a ROS 2 environment.

It loads a URDF, parses links into FCL shapes, checks for collisions (including self-collisions or with objects), and optionally visualizes collisions via RViz2 markers.

## Clone the Repository

```bash
git clone git@github.com:vanshgupta4452/fcl-ros2.git
```



##  Install Dependencies

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
  libassimp-dev
```



##  Build the Package

```bash
cd ~/ros2_ws
colcon build 
source install/setup.bash
```

## ▶ Run the Node

```bash
#environment collision 
ros2 run fcl_coll fcl_self_collision_node

#self collision
ros2 run fcl_coll self_node
```






