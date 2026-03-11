# Lidar Simulation Documentation

## Overview

The lidar simulation in PACSIM generates realistic 3D point clouds for a multi-channel lidar sensor. The simulation models a rotating lidar with multiple vertical channels that scans the environment, detecting both ground points and cone landmarks (blue, yellow, and orange cones).

## Table of Contents

1. [Architecture](#architecture)
2. [Configuration Parameters](#configuration-parameters)
3. [Point Cloud Generation Pipeline](#point-cloud-generation-pipeline)
4. [Mathematical Formulas](#mathematical-formulas)
5. [Implementation Details](#implementation-details)

---

## Architecture

The lidar simulation consists of the following main components:

- **Configuration Loading**: Reads lidar parameters from YAML configuration
- **Point Cloud Generation**: Main entry point that orchestrates the simulation
- **Occlusion Detection**: Determines which areas are blocked by cones
- **Floor Point Generation**: Simulates ground returns using ray tracing
- **Cone Point Sampling**: Generates points on cone surfaces based on distance

### Class Structure

```cpp
class lidarModel {
    uint32_t total_ray;           // Total number of rays the lidar can emit
    uint16_t points_per_arch;     // Horizontal resolution (angular bins)
    uint16_t num_channel;         // Number of vertical channels
    double min_angle_horizontal;  // Minimum horizontal angle (rad)
    double max_angle_horizontal;  // Maximum horizontal angle (rad)
};
```

---

## Configuration Parameters

### Default Configuration (from `config/lidar.yaml`)

```yaml
lidar:
  total_ray: 72000              # Total ray budget for the entire scan
  points_per_arch: 450          # Horizontal angular resolution
  num_channel: 28               # Vertical channels
  min_angle_horizontal: -1.0472 # -60° in radians
  max_angle_horizontal: 1.0472  # +60° in radians
```

### Parameter Meanings

- **total_ray**: The total number of rays available for the entire point cloud. This is used to calculate point density on detected objects.
- **points_per_arch**: Number of discrete horizontal angles sampled (azimuth resolution).
- **num_channel**: Number of vertical laser channels, similar to Velodyne or Ouster lidars.
- **min/max_angle_horizontal**: Horizontal field of view (FOV) in radians.

### Physical Constants

```cpp
#define X_CONE_DIM 0.228  // Cone width (m)
#define Y_CONE_DIM 0.228  // Cone depth (m)
#define Z_CONE_DIM 0.325  // Cone height (m)
#define RADIUS 0.114      // Cone base radius (m): X_CONE_DIM/2
#define LIDAR_Z 0.5       // Lidar height above ground (m)
```

These constants define the geometry of Formula Student cones and the lidar mounting height.

---

## Point Cloud Generation Pipeline

The `generatePointCloud()` method follows this sequence:

```
1. Initialize empty point cloud
2. Create occlusion array → fillOcclusionsArray()
3. Generate floor points → generateFloorPoints()
4. For each cone landmark:
   a. Calculate distance from lidar
   b. Compute cone surface area
   c. Calculate number of samples → sampleOnCone()
   d. Generate random points on cone → samplePointOnCone()
   e. Add colored points to cloud
5. Return complete point cloud
```

---

## Mathematical Formulas

### 1. Occlusion Detection

**Purpose**: Determine which directions are blocked by cones to prevent floor points from appearing through objects.

#### Angular Coverage Calculation

For each cone at position $(x, y, z)$:

**Distance from lidar:**
```
D = √(x² + y² + z²)
```

**Central angle (azimuth) to cone:**
```
θ = arctan2(y, x)
```

**Half-angle subtended by cone base:**
```
α = arcsin(R / D)
```
where $R = 0.114$ m is the cone base radius.

**Why this formula?** 
The half-angle α represents the angular size of the cone as seen from the lidar. Using the small angle approximation for circular objects, when viewing a circle of radius R at distance D, the angular radius is arcsin(R/D). This accounts for the 3D projection correctly.

#### Affected Angular Range

The cone blocks rays in the angular range:
```
θ_start = θ - α
θ_end = θ + α
```

#### Index Mapping

Convert continuous angles to discrete array indices:
```
i_start = ⌈(θ - α - θ_min) / (θ_max - θ_min) × (N - 1)⌉
i_end = ⌊(θ + α - θ_min) / (θ_max - θ_min) × (N - 1)⌋
```
where:
- $N$ = `points_per_arch`
- $θ_{min}$ = `min_angle_horizontal`
- $θ_{max}$ = `max_angle_horizontal`

**Why this mapping?**
This formula performs linear interpolation from continuous angle space to discrete array indices. The normalization factor `(θ - θ_min) / (θ_max - θ_min)` converts the angle to a [0,1] range, then multiplication by (N-1) maps it to indices [0, N-1].

#### Occlusion Distance

For each affected index:
```
occlusions[i] = min(occlusions[i], D - R)
```

The subtraction of R places the occlusion boundary at the cone's surface rather than its center.

---

### 2. Floor Point Generation

**Purpose**: Simulate lidar returns from the ground plane, accounting for occlusions.

#### Vertical Channel Distribution

For channel $k ∈ [0, N_{ch}-1]$:

**Vertical angle (elevation):**
```
φ_v = φ_min + (φ_max - φ_min) × k / (N_ch - 1)
```
where:
- $φ_{min} = -25°$ (downward)
- $φ_{max} = -0.3°$ (nearly horizontal)
- $N_{ch}$ = `num_channel`

**Why this range?**
This range is chosen to focus on ground detection. Channels are directed downward to capture floor returns, with the most horizontal channel at -0.3° to avoid looking completely flat (which would never hit the ground).

#### Ground Intersection Distance

For a lidar at height $H = 0.5$ m, a ray at vertical angle $φ_v$ intersects the ground at distance:

```
r = H / (-tan(φ_v))
```

**Derivation:**
Using basic trigonometry, if a ray travels at angle φ_v (negative for downward) from height H:
```
H = r × (-tan(φ_v))
∴ r = H / (-tan(φ_v))
```

The negative sign accounts for downward angles having negative values.

#### Point Coordinates

For horizontal angle $θ$ and ground distance $r$:
```
x = r × cos(θ)
y = r × sin(θ)
z = 0
```

#### Occlusion Check

A ground point is generated only if:
```
0 < r ≤ occlusions[i]
```

This ensures floor points don't appear behind cones.

---

### 3. Cone Surface Sampling

**Purpose**: Generate realistic point returns from cone surfaces with distance-dependent density.

#### Projected Surface Area

The visible surface area of a cone:
```
A_cone = (X_CONE_DIM × Z_CONE_DIM) / 2
A_cone = 0.228 × 0.325 / 2 = 0.03705 m²
```

**Why divide by 2?**
This approximates the average visible surface when viewing a cone from typical angles. A cone has both front and side surfaces, but typically only about half is visible from any given viewpoint due to self-occlusion.

#### Number of Sample Points

```
N_samples = ⌈R_total × A_cone / (2π D² sin(20°))⌉
```

where:
- $R_{total}$ = `total_ray` = 72000
- $A_{cone}$ = projected cone surface area
- $D$ = distance from lidar to cone
- $sin(20°) = sin(π/9) ≈ 0.342$

**Why this formula?**

This implements **angular density conservation**. The denominator $2π D² sin(20°)$ represents the surface area of a spherical cap at distance D with a 20° half-angle (typical vertical FOV).

The formula ensures:
1. **Inverse square law**: Points decrease with $D²$ as the same ray density spreads over larger areas
2. **Surface area proportionality**: Larger cones get more points
3. **Ray budget**: Total points across all objects respects the `total_ray` budget

The factor $sin(20°)$ accounts for the vertical field of view, converting from full sphere area ($4πD²$) to the relevant scanning region.

---

### 4. Point Position on Cone

**Purpose**: Distribute sample points realistically on cone surface with proper vertical weighting.

#### Vertical Channel Counting

Determine how many laser channels can hit the cone at distance D:

```
θ_threshold = arctan(H / D)
k_min = ⌈(θ_threshold - φ_min) / Δφ⌉
N_visible = N_ch - k_min
```

where:
- $Δφ = (φ_{max} - φ_{min}) / (N_{ch} - 1)$ is the angular spacing between channels
- $H = 0.5$ m (lidar height)
- $k_{min}$ is the first channel that can reach the cone base

**Why this calculation?**
Channels with elevation angles too steep will hit the ground before reaching the cone. The threshold angle is the geometric limit where a ray just grazes the ground at distance D.

#### Weighted Vertical Sampling

Points are sampled with linear probability weighting:

```
weight[k] = (N_visible - k) / Σ(N_visible - i)
```

where $k ∈ [0, N_{visible}-1]$

**Why decreasing weights?**
Lower parts of cones receive more hits because:
1. More laser channels can reach them (geometric visibility)
2. Ground-focused channels concentrate density at lower heights
3. This matches real lidar behavior

**Normalization:**
```
Σ weight[k] = 1
```

#### Height Discretization

The cone is divided into k discrete vertical levels:

```
Δz = Z_CONE_DIM / k
z_sample = z_cone_base + level × Δz
```

where `level` is randomly selected according to the weights above.

#### Cone Geometry Model

The cone is modeled as a truncated cone tapering from radius R at the base to a point:

**Radius at height z:**
```
r(z) = R × (1 - z / Z_CONE_DIM)
```

This linear taper matches Formula Student cone geometry.

#### Horizontal Sampling

**Angular range covered by cone:**
```
θ_cone = arctan2(y, x)           # Central angle to cone
α = arccos(R / D)                # Half-angle based on cone radius
```

**Sample angle range (back-facing):**
```
φ_min = π + θ_cone - α
φ_max = π + θ_cone + α
```

**Why π + θ_cone?**
We sample points on the back side of the cone (facing the lidar). Adding π rotates to the opposite side.

**Random angle:**
```
φ_sample ~ Uniform(φ_min, φ_max)
```

#### Final 3D Coordinates

```
x = x_cone + r(z_sample) × cos(φ_sample)
y = y_cone + r(z_sample) × sin(φ_sample)
z = z_sample
```

---

## Implementation Details

### Performance Optimizations

1. **Occlusion Pre-computation**: The occlusion array is computed once before floor generation, avoiding redundant calculations.

2. **Index Range Limiting**: Instead of checking all 450 horizontal bins, only indices within the cone's angular range are updated:
   ```cpp
   int start_idx = max(0, ⌈...⌉);
   int end_idx = min(points_per_arch - 1, ⌊...⌋);
   for (int i = start_idx; i <= end_idx; ++i) { ... }
   ```

3. **Fast Channel Counting**: `countChannelsFast()` uses direct calculation instead of iterating through all channels.

### Color Coding

Points are colored based on landmark type:
- **Blue cones**: RGB(0, 0, 255)
- **Yellow cones**: RGB(165, 173, 3)
- **Orange cones**: RGB(255, 165, 0)
- **Floor points**: RGB(128, 128, 128) - gray

### Random Number Generation

The implementation uses C++11 random number generators:
- `std::default_random_engine` for repeatability (can be seeded)
- `std::uniform_real_distribution` for uniform sampling on cone surface
- `std::discrete_distribution` for weighted vertical level selection

### PCL Integration

The output is a PCL (Point Cloud Library) point cloud:
```cpp
pcl::PointCloud<pcl::PointXYZRGB>
```

This is then converted to ROS2 `sensor_msgs::msg::PointCloud2` using:
```cpp
pcl::toROSMsg(cloud, cloudMsg);
```

### Frame of Reference

- **Coordinate system**: The lidar is at the origin, with the car's frame
- **Frame ID**: "car" (published in ROS2 messages)
- **Units**: All distances in meters, angles in radians

---

## Usage in Main Loop

From `pacsim_main.cpp`:

```cpp
if (perceptionSensor->getName() == "livox_front") {
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(lidarSensor->generatePointCloud(sensorLms, logger), cloudMsg);
    cloudMsg.header.frame_id = "car";
    cloudMsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(sensorLms.timestamp * 1e9));
    lidarPub->publish(cloudMsg);
}
```

The lidar simulation runs when:
1. A perception sensor named "livox_front" exists
2. The perception sensor ticks (based on its update rate)
3. The landmark list is available from the track

### Integration Flow

```
Track Landmarks → Perception Sensor Transform → 
   Lidar Model → Point Cloud → ROS2 Message → /pacsim/perception/lidar_pcd
```

---

## Validation and Testing

### Key Behaviors to Validate

1. **Distance falloff**: Point density should decrease with $1/D²$
2. **Occlusion**: No floor points behind cones
3. **Vertical distribution**: More points on lower cone sections
4. **Color accuracy**: Correctly colored based on landmark type
5. **Ray budget**: Total points should respect `total_ray` constraint

### Debug Features

The implementation includes commented-out PCD file saving:
```cpp
// pcl::io::savePCDFileASCII(filename, cloud);
```

Uncomment to save point clouds for offline analysis.

---

## Configuration Tuning

### Adjusting Point Density

**Increase overall density:**
```yaml
total_ray: 144000  # Double the points
```

**Increase angular resolution:**
```yaml
points_per_arch: 900  # Finer horizontal bins
num_channel: 56       # More vertical channels
```

### Adjusting Field of View

**Wider horizontal FOV:**
```yaml
min_angle_horizontal: -1.5708  # -90°
max_angle_horizontal: 1.5708   # +90°
```

**Different vertical range:**
Modify in `generateFloorPoints()`:
```cpp
double min_vert_angle = -30.0 * M_PI / 180.0;  // More downward
double max_vert_angle = 5.0 * M_PI / 180.0;    // Look slightly up
```

### Tradeoffs

- **Higher resolution** → More realistic but slower simulation
- **More channels** → Better vertical sampling but more computation
- **Larger total_ray** → Denser point clouds but may exceed hardware capabilities

---

## Future Enhancements

Potential improvements to the lidar simulation:

1. **Noise models**: Add Gaussian noise to point positions
2. **Intensity simulation**: Model surface reflectivity
3. **Range limits**: Enforce min/max range constraints
4. **Beam divergence**: Model finite beam width effects
5. **Motion distortion**: Account for vehicle motion during scan
6. **Multi-echo**: Simulate returns from semi-transparent objects
7. **Temporal simulation**: Model actual spinning mechanism with time-varying azimuth

---

## References

### Related Files

- **Implementation**: `src/lidarModel/lidarModel.cpp`
- **Header**: `include/lidarModel/lidarModel.hpp`
- **Configuration**: `config/lidar.yaml`
- **Main integration**: `src/pacsim_main.cpp`

### Mathematical Background

- **Inverse square law**: Basic physics of light/laser intensity
- **Solid angle**: Spherical geometry for angular coverage
- **Ray casting**: Computer graphics ray tracing principles
- **Probabilistic sampling**: Monte Carlo methods for surface sampling

### Lidar Technology

The simulation is inspired by:
- Livox HAP (Horizon, Avia, and Phantom series)
- Velodyne VLP-16/32
- Ouster OS1/OS2

These are commonly used in autonomous racing and robotics applications.

---

## Conclusion

The PACSIM lidar simulation provides a physics-based, computationally efficient model of a multi-channel rotating lidar. The key design principles are:

1. **Geometric accuracy**: Proper modeling of cone occlusions and ground intersections
2. **Density realism**: Distance-dependent point density following inverse square law
3. **Performance**: Optimized algorithms for real-time simulation
4. **Configurability**: Easy tuning for different lidar characteristics

The mathematical formulas ensure physical plausibility while remaining computationally tractable for real-time simulation in the racing scenario.
