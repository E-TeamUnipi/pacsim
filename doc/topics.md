# PacSim — ROS2 Topics Overview

This document lists the main ROS2 topics, message types and short descriptions used by the PacSim simulator.

## Publishers (what PacSim publishes)

- `/pacsim/velocity` — geometry_msgs::msg::TwistWithCovarianceStamped
  - Ground-truth vehicle velocity (linear and angular) stamped and with covariance. Frame: `car`.

- `/clock` — rosgraph_msgs::msg::Clock
  - Simulation clock published for nodes using simulated time.

- `/pacsim/track/visualization` — visualization_msgs::msg::MarkerArray
  - RViz markers to visualize the whole track (landmarks/cones).

- `/pacsim/track/landmarks` — pacsim::msg::Track
  - Ground-truth list of all track landmarks (all cones) in the `map` frame.

- `/pacsim/perception/<sensor_name>/landmarks` — pacsim::msg::PerceptionDetections
  - Per-sensor perception detections (cones) after FOV, noise and filters. `<sensor_name>` comes from `config/perception.yaml`.

- `/pacsim/perception/<sensor_name>/visualization` — visualization_msgs::msg::MarkerArray
  - Per-sensor marker array for RViz visualization of detected landmarks.

- `/pacsim/imu/<sensor_name>` — sensor_msgs::msg::Imu
  - Simulated IMU data (linear acceleration and angular velocity) for each configured IMU sensor (no valid orientation by default).

- `/pacsim/gnss/<sensor_name>` — pacsim::msg::GNSS
  - Simulated GNSS messages with latitude/longitude/altitude, ENU velocities and covariances (sensor-dependent noise).

- `/pacsim/steeringFront` — pacsim::msg::StampedScalar
  - Simulated steering front actuator (published value + stamp).

- `/pacsim/steeringRear` — pacsim::msg::StampedScalar
  - Simulated steering rear actuator (published value + stamp).

- `/pacsim/ts/voltage` — pacsim::msg::StampedScalar
  - Simulated telemetry: voltage time series.

- `/pacsim/ts/current` — pacsim::msg::StampedScalar
  - Simulated telemetry: current time series.

- `/pacsim/wheelspeeds` — pacsim::msg::Wheels
  - Published simulated wheel speeds (FL/FR/RL/RR).

- `/pacsim/torques` — pacsim::msg::Wheels
  - Published simulated wheel torques (FL/FR/RL/RR).

- `/pacsim/perception/lidar_pcd` — sensor_msgs::msg::PointCloud2
  - Point cloud generated from the simulated lidar (published when lidar sensor produces a cloud).

- `/joint_states` — sensor_msgs::msg::JointState
  - Joint states for vehicle wheels/steering (useful for visualizers/robot_state_publisher).


## Subscribers (what PacSim listens to)

- `/pacsim/steering_setpoint` — pacsim::msg::StampedScalar
  - External steering setpoint input (used by simulator control callbacks).

- `/pacsim/torques_min` — pacsim::msg::Wheels
  - Minimum torque limits for the motor controllers.

- `/pacsim/torques_max` — pacsim::msg::Wheels
  - Maximum torque limits for the motor controllers.

- `/pacsim/wheelspeed_setpoints` — pacsim::msg::Wheels
  - Wheelspeed setpoints for each wheel.

- `/pacsim/powerground_setpoint` — pacsim::msg::StampedScalar
  - Power ground setpoint input.


## Services

- `/pacsim/finish_signal` — std_srvs::srv::Empty
  - Service to signal the simulator to finish/stop the run.

- `/pacsim/clock_trigger/absolute` — pacsim::srv::ClockTriggerAbsolute
  - Set an absolute simulation stop time.

- `/pacsim/clock_trigger/relative` — pacsim::srv::ClockTriggerRelative
  - Set a simulation runtime relative to the current sim time.


## TF / frame broadcasting

- TF broadcaster (transform between track/map and vehicle): PacSim broadcasts a transform (TF2) with child frame `car` and parent `trackFrame` (commonly `map`). Use a TF2 listener to read exact vehicle pose (ground truth).

- Optional static transforms: If `broadcast_sensors_tf2` is enabled in the main config, PacSim publishes static transforms for each sensor frame (e.g. IMU, lidar, perception frames) with parent frame `cog_frame_id_pipeline`.


## Notes & patterns

- Perception and sensor topics are dynamically created per configured sensor. Common patterns:
  - Perception detections: `/pacsim/perception/<sensor_name>/landmarks`
  - Perception visualization: `/pacsim/perception/<sensor_name>/visualization`
  - IMU: `/pacsim/imu/<sensor_name>`
  - GNSS: `/pacsim/gnss/<sensor_name>`

- Ground-truth vehicle pose is best read via TF2 (map/track -> `car`). Velocity is published on `/pacsim/velocity`.

- If you want a single topic combining pose+twist (odometry), PacSim currently does not publish `nav_msgs/Odometry` by default; you can either listen to TF2 + `/pacsim/velocity` or add a small node that listens to TF and `/pacsim/velocity` and republishes `nav_msgs/Odometry`.

---
Generated from PacSim source (main node `pacsim_main.cpp`) — concise summary of topics and roles.