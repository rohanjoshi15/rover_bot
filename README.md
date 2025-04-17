# ROS 2 Lidar-Based Emergency Stop System

This system uses a **LiDAR sensor** to detect obstacles, calculates the minimum distance in front, issues a warning if the object is too close, and triggers an **emergency stop**.

## Overview

The system is composed of **three ROS 2 nodes**:
1. `LidarProcessor`: Subscribes to LiDAR data (`/scan`), publishes the front distance to `/min_distance`.
2. `ProximityWarning`: Subscribes to `/min_distance`, checks if the robot is too close to an obstacle and publishes a warning to `/proximity_warning`.
3. `EmergencyStop`: Subscribes to `/proximity_warning`, and if a stop signal is received, publishes zero velocity to `/cmd_vel`.

---

## Node 1: `LidarProcessor`

### Function:
- Subscribes to sensor_msgs/msg/LaserScan on topic /scan
- Publishes std_msgs/msg/Float32 to topic /min_distance

## Node 2: ProximityWarning
### Function:
- Subscribes to std_msgs/msg/Float32 on topic /min_distance
- Publishes std_msgs/msg/String to topic /proximity_warning if distance ≤ 1 meter

## Node 3: EmergencyStop
### Function:
- Subscribes to std_msgs/msg/String on topic /proximity_warning
- Publishes geometry_msgs/msg/Twist to topic /cmd_vel to stop the robot

---
# QoS Settings for Each Topic

Quality of Service (QoS) settings in ROS 2 determine how data is sent between publishers and subscribers.
QoS with depth 10 here have over-filled queues

---

## 1. /scan  
- Publisher: LiDAR driver   
- Subscriber: LidarProcessor node  
- QoS Settings:
  - Reliability: RELIABLE 
  - Durability: VOLATILE 
  - History: KEEP_LAST
  - Depth: 10

## 2. /min_distance
- Publisher: LidarProcessor node
- Subscriber: ProximityWarning
- QoS Settings:
  - Reliability: RELIABLE
  - Durability: VOLATILE
  - History: KEEP_LAST
  - Depth: 10

## 3. proximity_warning

- Publisher: ProximityWarning node  
- Subscriber: EmergencyStop node  
- QoS Settings:
  - Reliability: RELIABLE  
  - Durability: VOLATILE` 
  - History: KEEP_LAST
  - Depth: 10

## 4. /cmd_vel

- Publisher: EmergencyStop node  
- Subscriber: Robot base controller  
- QoS Settings:
  - Reliability: reliable 
  - Durability: volatile  
  - History: Keep last
  - Depth: 10

## 5. /camera/image/raw 
- Depth: 5
- Durability Policy: Volatile
- History Policy: Keep Last
- Reliability Policy: Best Effort

## 6. robot_description from rviz
- Depth: 1
- Durability Policy: Transient Local
- History Policy: Keep Last
- Reliability Policy: Reliable




---
## Launch FIle 
This ROS 2 launch file does the following:

1. **Loads robot model**:
   - Processes a `.xacro` file to generate the URDF robot description.

2. **Starts Gazebo simulation**:
   - Includes the default Gazebo launch file and loads a custom world.

3. **Spawns the robot in Gazebo**:
   - Uses `spawn_entity.py` to insert the robot using the robot description.

4. **Publishes robot state**:
   - Starts `robot_state_publisher` to publish joint and TF data from URDF.
   - Optionally runs `joint_state_publisher` (though it’s not added to the launch list).

5. **Starts RViz2**:
   - Opens RViz with a predefined config file for visualizing the robot and data.


---
Open another terminal 
---

Launch the file from the rover_controller package

This launch file starts the control logic nodes of the rover:

1. **`lidar_processor_node`**:
   - Subscribes to `/scan` (LaserScan)
   - Publishes the minimum distance as a `Float32` to `/min_distance`  Here float32 is for precision 
Also logic could be like taking the middle value from the array of values the lidar provides

2. **`proximity_warning_node`**:
   - Subscribes to `/min_distance`
   - Publishes a warning message (like `'STOP'`) to `/proximity_warning` if the distance is too small

3. **`emergency_stop_node`**:
   - Subscribes to `/proximity_warning`
   - Sends a zero-velocity `Twist` message to `/cmd_vel` to stop the robot when `'STOP'`
