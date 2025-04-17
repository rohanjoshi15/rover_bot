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
