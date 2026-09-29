# Nav2 MPC controller

Differential-drive controller for ROS 2 Navigation2, using a generated
ACADOS solver.

## MPC model

- State: `[x, y, yaw, linear_velocity, angular_velocity]`
- Input: `[linear_acceleration, angular_acceleration]`
- Horizon: 50 nodes
- Runtime obstacle parameters: 10 `(x, y)` points in the current robot frame,
  followed by one shared clearance radius

Laser and optional PointCloud2 points are transformed into the costmap global
frame when received and back into the current robot frame immediately before
each solve. This prevents sensor mounting offsets and robot motion between
sensor and control cycles from corrupting the ACADOS obstacle coordinates.

The MPC initial linear and angular velocities are calculated from consecutive
odometry poses because this platform does not provide usable odometry twist.

## Obstacle parameters

Parameters are under the configured controller plugin name (for example,
`FollowPath`).

```yaml
FollowPath:
  use_obstacle_constraints: true
  treat_unknown_as_obstacle: true
  scan_topic: /scan
  use_pointcloud_constraints: true
  pointcloud_topic: /perception/output
  odom_topic: /odom
  odom_velocity_min_dt: 0.1
  odom_timeout: 0.5
  odom_velocity_filter_alpha: 1.0
  use_curvature_speed_scaling: true
  min_curve_velocity: 0.15
  curvature_speed_safety_factor: 1.0
  curvature_lookahead_distance: 0.2
  obstacle_constraint_radius: 0.20
  obstacle_max_range: 3.0
  obstacle_min_separation: 0.15
  obstacle_min_angle: -1.5708
  obstacle_max_angle: 1.5708
  scan_timeout: 0.5
  pointcloud_max_range: 3.0
  pointcloud_min_height: -0.2
  pointcloud_max_height: 1.0
  pointcloud_timeout: 0.5
  collision_stop_distance: 0.5
  collision_slowdown_factor: 0.8
  linear_accel_min: -2.0
  linear_accel_max: 2.0
  angular_accel_min: -3.0
  angular_accel_max: 3.0
```

`obstacle_constraint_radius` is the clearance used by the generated circular
point-obstacle constraint. Increase it only after confirming that the physical
footprint and laser frame are calibrated. The generated obstacle constraints
are hard constraints; an infeasible solve results in a zero velocity command.

The reference path is sampled with a curvature-aware speed profile. When
`use_curvature_speed_scaling` is enabled, the controller lowers the reference
velocity before high-curvature segments using the angular velocity limit,
`curvature_speed_safety_factor`, and `curvature_lookahead_distance`. This keeps
the MPC reference from running too far ahead on tight curves.

When `use_pointcloud_constraints` is enabled, the controller subscribes to
`pointcloud_topic` as `sensor_msgs/msg/PointCloud2`. Valid pointcloud points are
filtered by sensor-frame range and global-frame height, then merged with fresh
laser points. The final MPC obstacle list is selected in the current robot frame
by distance, angle window, and `obstacle_min_separation`, so the 10 generated
ACADOS obstacle slots are shared by all enabled sensor sources.

For RViz debugging, selected obstacle points are published on
`/mpc_controller/selected_obstacles`. Red spheres show the points sent toward
the MPC parameter vector, and translucent red disks show the configured
`obstacle_constraint_radius`.

Laser scan angles are normalized before applying `obstacle_min_angle` and
`obstacle_max_angle`, so front-window filters such as `[-pi/2, pi/2]` also work
with scans published over `[0, 2pi]`.
