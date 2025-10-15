A **PID Controller** (Proportional-Integral-Derivative) is a common way to control a systems output to match a desired target.

In FT, we use a PID for speed control. It adjusts the **acceleration (or throttle/brake)** so that the car's **actual velocity** matches the **target velocity.**

Essentially, if the car is *slower than the target*, **accelerate** or if the car is *faster than the target*, **brake**. We must do this **smoothly** and not **overshoot.**

We do this by continuously measuring the **error** between what we have and what we want:$$error(t)\  = \  v_{target}(t)\  - \  v_{actual}(t)$$
- Where $t$ denotes the current unit time.

**Formula:**
$$u(t) \  = \  K_pe(t)\  + \  K_i \int^t_0 e(\tau)d\tau\  +\  K_d\frac{de(t)}{dt}$$
Where:
- $u(t)$: control output (acceleration command)
- $e(t)$: current error
- $K_p, K_i, K_d$: tuning gains

| Term                 | Meaning                           | Behavior                                                   |
| -------------------- | --------------------------------- | ---------------------------------------------------------- |
| **P (Proportional)** | Reacts to current error           | Large Kp → fast response, but may overshoot                |
| **I (Integral)**     | Reacts to accumulated past error  | Fixes steady-state offset, but may cause drift/oscillation |
| **D (Derivative)**   | Reacts to rate of change of error | Dampens oscillation, smooths motion                        |
```cpp
float PID(float error) {
	integral += error * dt;
    derivative = (error - prev_error) / dt;
    output = Kp * error + Ki * integral + Kd *
    derivative;
    prev_error = error;
    return output;
}
```

This **output** is clamped by *pure_pursuit.cpp* between `max_accel` and `-max_accel` before publishing.

A general rule of thumb with race-cars: **$Kp$ dominates, $Ki$ small, $Kd$ mid.**

**Tuning Issue Guide:**

| Problem                              | Likely Cause         | Fix                        |
| ------------------------------------ | -------------------- | -------------------------- |
| Car oscillates speed up/down         | $Kp$ too high        | Reduce $Kp$ or add $D$     |
| Car never reaches target speed       | $Kp$ too low         | Increase $Kp$              |
| Car slowly drifts above below target | $Ki$ too low         | Add small $Ki$             |
| Car reacts sluggishly                | $Kd$ too high        | Reduce $Kd$                |
| Car jitters with noise               | Noisy velocity input | Add filter or reduce gains |


