Simply put, *pure_pursuit.py* and *pure_pursuit.cpp* are two [[ROS (Robot Operating Software)]] files which make up the two implementations of Formula Trinity's AI Control.

These [[ROS (Robot Operating Software)|ROS]] nodes listen for the car's pose (orientation and location of the car, *car_x, car_y, quaternion -> car_yaw*) and a stream of race-line waypoints, picks a **goal point** a short distance in front of the car (the "lookahead" circle), computes a **steering angle** to reach that goal, and publishes an Ackermann **drive command** (steer + accel).

We **subscribe** to:
- **/pose**: where the car is and how it is oriented. Used to set `car_x`, `car_y`, `car_yaw`.
- **/raceline** (`PointArray`): the sequence of (x, y) waypoints to follow.
- Speed Source: either **/odometry_integration/car_state** for sim velocity or **/ros_can/wheel_speeds** for rear-wheel based speed velocity.
- Control overrides from the state machine: **/state_machine/safety_control** and **/state_machine/mission_control** (stop/crawl/target velocity/accel).

We **publish**:
- **/cmd** (`AckermannDriveStamped`): the actual steering angle and acceleration to the vehicle.
- **/steering_angle** (Pose for viz) and **/distance** (how fare you've travelled). The C++ version also publishes a marker for the goal point.

**Important Parameters:**
- `use_slam` (bool): read pose from SLAM frames vs base_footprint; sets `frame_id`.
- `nominal_speed`, `crawl_speed` (m/s): baseline speeds.
- `linear_speed_source`: **"sim"** or **"wheel_speeds"**.
- `wheel_radius` (m): used to convert wheel rpm to m/s.
- Lookahead distance `ld`: computed as `L * coefficient` (wheelbase * multiplier). In *pure_pursuit.py* it's a fixed `self.L * 1.2`. In *pure_pursuit.cpp* it's parameterized via `ld_coefficient`. **This is the #1 fine-tuning knob.**

**The [[Pure Pursuit]] Loop:**
1. **Read State**:
	- From **/pose**: `(car_x, car_y, car_yaw)` Yaw is extracted from the quaternion -> angle helper. 
2. **Pick the 2 waypoints around your lookahead circle:**
	- Compute distance from car to each race-line waypoint
	- Find the **nearest waypoint** ("inside" the circle) and then the next waypoint **just outside** the lookahead radius `ld`. These two define a line that crosses your lookahead circle.
3. **Find the circle-line intersections:**
	- The line through those two intercepts is intersected with the lookahead circle around the car -> **two intercepts**. Pick whichever is **closer to the outside waypoint;** that becomes the **goal point**.
4. **Turn towards the goal:**
	- Compute $\alpha$ **(alpha)**: angle between the current heading and the vector from car -> goal.
	- **Steering Angle** = $atan(2*L*\sin(\alpha)\  /\  ld)$. *Pure Pursuit Formula*
5. **(C++ Only) Grip Aware Helpers (beta/gamma/tau) + LAP:**
	- C++ has an extra layer: it estimates curvature, mixes a **gamma** (how far you are from the path) and **beta** (curvature gap) to nudge the goal (a **Look-Ahead Point, LAP**) and then re-computes the steering towards the LAP for smoother path-following. Think "pure_pursuit + correction for path curvature/offset".
6. **Decide acceleration:**
	- If safety/mission says **stop/crawl/follow target vel/accel**, those override.
	- Otherwise, compute a target velocity (simple rules in C++; nominal in Python) and drive a tiny P/PI/PID to move actual velocity towards it, **clamped** by `max_accel`. (Python uses simple proportional choices, C++ has PID scaffold).
7. **Publish**
	- Send **/cmd** with `steering_angle` and `acceleration`. Update **/distance** by integrating velocity over time. Publish steering/goal viz.


**Glossary:**
- **Ackermann**: car-like steering (front-wheels turn), so we command **steering angle** and **acceleration**.
- **Lookahead (ld)**: radius of circle centered at the car; we aim for a point where the raceline crosses the circle. Bigger **ld** looks further ahead.
- **Alpha (α)**: the heading error between where the car points and where the goal point is.
- **LAP / beta / gamma** (C++): small add-ons to pure pursuit to account for curvature and how far off the path you are; they slightly shift the goal point for smoother tracking.


Speed control is performed using a [[PID Control|PID Controller]].