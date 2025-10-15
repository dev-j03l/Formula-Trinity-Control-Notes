**Overview:**
Pure Pursuit is a **geometric path-tracking algorithm** used in autonomous vehicles to compute the steering angle needed to follow a predefined path of waypoints.

It belongs to the **[[Control Overview|Control]]** Layer of the stack, taking in *where the car is* (pose) and *where it should go* (raceline), and outputting steering commands.

The car constantly looks ahead a distance `ld` (the **lookahead distance**) on the raceline.
The point where the raceline intersects a circle of radius `ld` centered on the car is called the **goal point**.

By steering toward this point, the car naturally follows the path in smooth arcs.

**Mathematics Summary:**
Let:
- $(x,y)$ = current car position
- $\theta$ = current car heading (yaw)
- $L$ = wheelbase (distance between front and rear axles)
- $ld$ = lookahead distance
- $\alpha$ = angle between car heading and line to goal point

Then the **steering angle** $\delta$ is given by:
$$\delta \  = \  \arctan(\frac{2L\sin (\alpha)}{ld})$$
