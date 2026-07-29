# Technical Documentation: LiDAR Simulation Algorithm

## 1. Introduction
This document provides a detailed technical description of the LiDAR simulation algorithm implemented in the `lidarModel` class. The primary objective of this module is to generate a realistic 3D point cloud (`pcl::PointCloud<pcl::PointXYZRGB>`) that simulates the output of a physical scanning LiDAR sensor. The simulation detects landmarks (e.g., track cones) and the ground plane, accounting for sensor specifications, scanning dynamics, and object occlusions.

## 2. Reference Frames
To accurately simulate the sensor and represent points in space, the algorithm operates between two primary coordinate frames:

- **Vehicle/Base Frame**: The local coordinate system of the vehicle. The input landmarks (cones) are provided by the simulator in this frame. The axes follow standard conventions (e.g., X forward, Y left, Z up).
- **LiDAR Sensor Frame**: The local coordinate system attached directly to the LiDAR sensor. The origin of this frame is translated from the Vehicle Frame by the offset parameters `(lidar_x, lidar_y, lidar_z)`. The generated output point cloud is represented in this LiDAR Sensor Frame. For example, a landmark point $(X_{base}, Y_{base}, Z_{base})$ in the base frame is translated to $(X_{base} - lidar\_x, Y_{base} - lidar\_y, Z_{base} - lidar\_z)$ to be properly relative to the LiDAR.

## 3. Configuration Parameters
The behavior of the `lidarModel` is controlled by parameters loaded during initialization (e.g., from the `lidar.yaml` configuration file).

| Parameter | Description |
| :--- | :--- |
| `total_ray` | Total number of rays emitted by the sensor per full 360° scan or full cycle. |
| `points_per_arch` | Horizontal resolution, defining the number of discrete horizontal angles for the occlusion buffer and ground generation. |
| `num_channel` | Number of vertical channels (laser beams) of the sensor. |
| `num_segments` | The number of segments into which the horizontal Field of View (FOV) is divided. Used to simulate progressive scanning over time and motion distortion. |
| `rate` | Scanning frequency in Hz. |
| `min_angle_horizontal` | The minimum bound of the horizontal FOV (in radians). |
| `max_angle_horizontal` | The maximum bound of the horizontal FOV (in radians). |
| `angular_uncertainty` | The angular measurement uncertainty, used as the standard deviation for Gaussian noise generation. |
| `lidar_x`, `lidar_y`, `lidar_z` | Translation of the LiDAR Sensor Frame relative to the Vehicle Frame. |
| `perception_sensor_name` | Identifier string for the sensor (e.g., "livox_front"). |

## 4. Algorithm Execution (`RunTick`)
The simulation updates in discrete time steps, mimicking the sensor's physical scanning process. 

### 4.1. Timing and FOV Calculation
The algorithm calculates the time required to scan the active FOV and any potential "dead time":
- **Active Time ($t_{active}$)**: Time spent actively scanning the FOV bounds (`max_angle_horizontal` - `min_angle_horizontal`).
- **Segment Time ($dt_{segment}$)**: The active time divided by `num_segments`.
- **Dead Time ($t_{dead}$)**: Time spent rotating outside the active FOV (relevant for sensors that mechanically spin but only have a directional FOV, like a Livox Front).

### 4.2. Segment Acquisition
At each `RunTick`, if enough time ($dt_{segment}$) has passed, the algorithm processes the next horizontal segment:
- It queries the internal `PerceptionSensor` for landmarks visible at the current simulation time.
- It calls `generateSegment` to process these landmarks and generate the corresponding portion of the point cloud.

### 4.3. Latency Simulation
Once all segments forming the active FOV are processed, the algorithm simulates latency by waiting for the dead time ($t_{dead}$) to elapse before publishing the fully aggregated point cloud.

## 5. Point Cloud Generation (`generateSegment`)
For a given segment of the horizontal FOV (bounded by angles `phi_start` and `phi_end`), the algorithm performs the following sequential steps:

### 5.1. Occlusion Buffer Construction (`fillOcclusionsArray`)
To properly simulate visibility and shadowing (rays blocked by closer objects), the algorithm maintains a 1D depth buffer (`occlusions` array) for the current segment.
- **Initialization**: The buffer is initialized to a maximum distance for the current segment's angular indices.
- **Update**: It iterates over the cones in the segment, calculates the horizontal angle subtended by each cone, and updates the buffer with the minimum distance (distance to the cone's surface) for the covered angles.

### 5.2. Ground Plane Generation (`generateFloorPoints`)
The algorithm generates ground returns by simulating the vertical channels.
- **Iteration**: For each horizontal angle in the current segment and each vertical channel, it computes the intersection of the ray with the ground plane (where $Z = -lidar\_z$ in the LiDAR Frame).
- **Occlusion Check**: The horizontal distance to this ground intersection is compared against the `occlusions` depth buffer. If the ground intersection is closer than the occluding object, a ground point (grey color) is successfully added. If the object is closer, the ray is blocked and no ground point is generated.

### 5.3. Cone Surface Sampling (`sampleOnCone` & `samplePointOnCone`)
For each valid landmark (Blue, Yellow, or Orange cone) inside the segment, the algorithm simulates laser hits on its conical shape.
- **Sample Count**: Determines how many rays hit the cone based on its flattened surface area and inverse-square distance ($1/d^2$).
- **Z-Height Discretization**: The cone's vertical axis is discretized based on the number of vertical channels covering the object. A weighted discrete distribution favors hitting the wider, lower parts of the cone.
- **Point Sampling**: For a sampled Z-height, the cone's radius at that height is calculated, and a point is uniformly sampled on the facing perimeter of the cone.
- **Coordinate Transformation**: The sampled point is transformed into the LiDAR Sensor Frame (`sample_x - lidar_x`, etc.).
- **Segment Filtering**: A point-wise check ensures the generated point strictly falls within the angular bounds (`phi_start` to `phi_end`) of the current segment.

### 5.4. Noise Application (`applyNoise`)
After all segments are acquired and aggregated into a complete scan:
- The algorithm iterates through every generated point in the cloud.
- It calculates a standard deviation proportional to the point's distance and the configured `angular_uncertainty`.
- Gaussian noise is applied independently to the X, Y, and Z coordinates, simulating realistic sensor inaccuracies.

## 6. ROS Integration and Output
The final result is a structured `pcl::PointCloud<pcl::PointXYZRGB>` containing:
- Ground points representing the track surface.
- Cone points colored according to their landmark type (Blue, Yellow, Orange).
- Realistic density decay over distance.
- Occlusion shadows behind cones.
- Motion-like progressive scanning artifacts and latency.
- Simulated measurement noise.

This point cloud is subsequently converted and published to the ROS 2 network.
- **Topic**: `/pacsim/perception/lidar_pcd`
- **Message Type**: `sensor_msgs/msg/PointCloud2`
- **Frame ID**: `lidar`
