# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Model Description
The state is described by the following variables:
* `x` and `y` cartesian coordinates in the vehicle's frame.  `(0,0)` is the vehicle's position on receipt of new sensor data.
* `psi` is the vehicle's orientation.  `psi` is 0 at the origin (when new sensor data is received.)
* `v` is the vehicle's speed.
* `cte` is the cross-track-error: the lateral distance to the reference trajectory.
* `epsi` is the angular difference between `psi` and the tangent of the reference trajectory at `x`.

The actuators are:
* `a` is the throttle value.
* `delta` is the steering angle.

The state update equations are simple kinematic update equations that assume instaneous throttle response and steering response.

## Prediction
The trajectory is predicted over 1.5 seconds with a 0.1 second timestep.  A short time horizon was required because the relatively small number of waypoints returned from the simulator caused the computed reference trajectory to only be accurate in the vicinity of the vehicle.

## Latency Handling
Latency was accomodated by describing the intiial state of the vehicle to the optimization as a vehicle's position projected by the 100ms latency value into the future.  This made the model more robust against an initial position error that grew as the vehicle speed increased.
