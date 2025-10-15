ROS2 (Robot Operating System 2) is the **middleware** that connects all the car’s software components — sensors, perception, planning, and control — into one coordinated system.

It handles:
- **Communication** between nodes (publish/subscribe)    
- **Message passing** (topics)
- **Parameter management**  
- **Time synchronization** 
- **Lifecycle & launch management**

In short:
- Each module (e.g. perception, path-planning, control) is a **ROS2 node**, and they communicate through **topics** using predefined **message types**.

**Core ROS Concepts:**

| Concept            | Description                                | Example in Control              |
| ------------------ | ------------------------------------------ | ------------------------------- |
| **Node**           | A running program that performs one task   | `pure_pursuit`                  |
| **Topic**          | A named communication channel for messages | `/pose`, `/raceline`, `/cmd`    |
| **Publisher**      | Sends messages on a topic                  | Control node → `/cmd`           |
| **Subscriber**     | Receives messages from a topic             | Control node ← `/pose`          |
| **Message**        | Data format shared between nodes           | `geometry_msgs/msg/PoseStamped` |
| **Parameter**      | Config value loaded at startup             | `ld_coefficient: 1.2`           |
| **Launch file**    | Starts multiple nodes with params          | `control_launch.py`             |
| **TF (Transform)** | Tracks coordinate frames                   | `map → base_link`               |
| **rclcpp**         | C++ API for ROS2                           | Used in `pure_pursuit.cpp`      |


**Topics in the Control Stack:**

| Topic                                                | Direction                | Message Type                           | Purpose                        |
| ---------------------------------------------------- | ------------------------ | -------------------------------------- | ------------------------------ |
| `/pose`                                              | **Subscribed**           | `geometry_msgs/PoseStamped`            | Car’s position and orientation |
| `/raceline`                                          | **Subscribed**           | `eufs_msgs/PointArray`                 | Waypoints to follow            |
| `/wheel_speeds` or `/odometry_integration/car_state` | **Subscribed**           | `eufs_msgs/CarState`                   | Actual velocity                |
| `/state_machine/safety_control`                      | **Subscribed**           | `ft_msgs/StateControl`                 | Stop/crawl overrides           |
| `/cmd`                                               | **Published**            | `ackermann_msgs/AckermannDriveStamped` | Steering + accel commands      |
| `/goal_point`, `/steering_angle`, `/distance`        | **Published (optional)** | Various                                | Visualization/debug            |


**Code Examples:**

```cpp
class PurePursuit : public rclcpp::Node {
public:
  PurePursuit() : Node("pure_pursuit") {
    // Declare parameters
    declare_parameter("ld_coefficient", 1.2);

    // Create publishers/subscribers
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        /pose", 10, std::bind(&PurePursuit::pose_callback, this, _1));

    raceline_sub_ = create_subscription<eufs_msgs::msg::PointArray>(
        "/raceline", 10, std::bind(&PurePursuit::raceline_callback, this, _1));

    cmd_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
  }
};

```


- ROS2 is the **communication backbone** of the car.
- **Nodes** = programs, **topics** = message channels.
- Control = subscribing to sensors & planners → publishing drive commands.
- Use **YAML Parameters** to tune behavior without touching code. 
- ROS2 Tutorials: docs.ros.org/en/humble/Tutorials
- Ackermann Steering in ROS2: [`ackermann_msgs`](https://github.com/ros-drivers/ackermann_msgs)
