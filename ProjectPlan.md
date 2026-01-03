# Project plan

Autonomous Differential Drive Mobile Robot integration

- URDF & RViz: This is Kinematics & Geometry. You are defining the rigid body constraints and the static transforms (Tbase_linkwheel​).

- Gazebo: This is Dynamics & Physics. You are adding mass, inertia matrices (I), and collision properties to the geometry.

- Diff Drive Plugin: This is Control Theory. You are mapping velocity commands (v,ω) to wheel angular velocities (θ˙L​,θ˙R​) based on the robot's physical constraints (wheel radius, separation).

- Mapping (SLAM): This is Sensor Fusion. You are converting raw Lidar data into an occupancy grid.

- AMCL & Nav2: This is Probabilistic State Estimation. You are solving the "Kidnapped Robot Problem" using particle filters.

## High level tasks

- create URDF
  - visualize in Rviz
  - visualize in Gazebo

- robot state publisher
- joint state publisher

- gazebo plugin for diff drive
  - odom
  - odom to base link
  - joint state
  - scan

- navigation
  - create map
  - localization
  - navigation

## Implementation and steps

### URDF
| Component      | Variable | Value (Meters) | Description                                         |
| -------------- | -------- | -------------- | --------------------------------------------------- |
| Chassis Length | xbase​   | 0.30 m         | The front-to-back length.                           |
| Chassis Width  | ybase​   | 0.20 m         | The side-to-side width.                             |
| Chassis Height | zbase​   | 0.10 m         | The vertical thickness of the body.                 |
| Wheel Radius   | r        | 0.05 m         | 5 cm radius (10 cm diameter).                       |
| Track Width    | L        | 0.25 m         | Distance between the centers of the two wheels.     |
| Caster Radius  | rcaster​ | 0.03 m         | Smaller than drive wheels to ensure ground contact. |

#### create package

```bash
ros2 pkg create --build-type ament_cmake  my_robot_description
cd src/my_robot_description
mkdir urdf meshes launch config worlds
```

#### create URDF

#### create launch file

#### build

```bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

#### launch

```bash
ros2 launch my_robot_description rsp.launch.py
ros2 run rviz2 rviz2
```

#### gazebo information

- add <collision> and <intertial> properties to all links in urdf
- add gazebo reference to all the links