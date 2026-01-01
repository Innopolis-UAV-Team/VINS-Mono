# CHANGELOG

## [Unreleased] - 2026-01-01/02

### Added - Feature Detection & Tracking

#### AGAST Corner Detector
- **Replaced Shi-Tomasi with AGAST** (Adaptive Generic Accelerated Segment Test)
  - Modern, faster corner detection algorithm
  - Configurable threshold via `fast_threshold` parameter (default: 20)
  - Adaptive feature detection for varying lighting conditions

#### Grid-Based Feature Selection
- **Uniform spatial distribution** of features across image
  - Divides image into grid cells of size `MIN_DIST`
  - Selects strongest feature per grid cell
  - Memory-efficient implementation using `std::unordered_map`
  - Prevents feature clustering in high-texture areas

#### Bidirectional Optical Flow
- **Forward-Backward consistency check** for robust tracking
  - Forward flow: current → next frame
  - Backward flow: next → current frame  
  - Outlier rejection when error > 0.7 pixels
  - Configurable via `use_bidirectional_flow` parameter (0/1)
  - Can be disabled for performance on resource-constrained systems

#### Sub-Pixel Refinement Optimization
- **Smart refinement strategy** to reduce computational cost
  - Applies sub-pixel refinement only to top-50 strongest features
  - Reduces processing time while maintaining accuracy
  - Uses OpenCV `cornerSubPix` with TermCriteria (40 iterations, 0.001 epsilon)

#### CLAHE Optimization
- **Contrast Limited Adaptive Histogram Equalization** performance improvement
  - CLAHE object created once in constructor, reused per frame
  - Eliminates redundant object allocation overhead
  - Critical for thermal imaging with low contrast

### Added - Visual-Inertial Odometry

#### Zero Velocity Update (ZUPT)
- **Drift prevention during stationary periods**
  - Detects when camera/IMU is stationary
  - Resets velocity to zero to prevent IMU integration drift
  - Configurable parameters:
    - `enable_zupt`: 0/1 to disable/enable
    - `zupt_vel_threshold`: velocity threshold (m/s) for detection
    - `zupt_acc_threshold`: acceleration deviation threshold (m/s²)
  - Applied in `processIMU()` when velocity < threshold AND acceleration ≈ gravity

### Added - Configuration & Launch Files

#### Thermal Camera Configuration
- **Complete thermal camera support** (`config/termal_cam_config.yaml`)
  - Resolution: 384×288 pixels
  - Camera model: PINHOLE
  - Intrinsics: fx=382.17, fy=381.92, cx=177.60, cy=137.18
  - Distortion coefficients: k1=-0.4137, k2=0.1618, p1=-0.0004, p2=-0.0022
  - IMU-Camera extrinsics (from Kalibr calibration)
  - Rotation matrix: 90° CCW around Z-axis with axis corrections
  - Feature tracker tuning for thermal imagery:
    - max_cnt: 180 features
    - min_dist: 30 pixels (reduced clustering)
    - fast_threshold: 40 (stronger corners only)
    - F_threshold: 2.0 pixels (RANSAC outlier rejection)
    - equalize: 1 (CLAHE enabled)
    - use_bidirectional_flow: 1
  - IMU parameters:
    - acc_n: 0.1 (measurement noise)
    - gyr_n: 0.05 (measurement noise)
    - acc_w: 0.002 (bias random walk)
    - gyr_w: 4.0e-5 (bias random walk)
  - ZUPT configuration:
    - enable_zupt: 1
    - zupt_vel_threshold: 0.2 m/s
    - zupt_acc_threshold: 0.2 m/s²

#### Launch File for Thermal Camera
- **Created** `vins_estimator/launch/termal_cam.launch`
  - Configured for `/thermal_cam/image_raw` topic
  - Uses thermal camera config file
  - Ready for deployment

### Modified - Feature Tracking Visualization

#### Depth-Based Feature Size Rendering
- **Visual debugging enhancement** in `feature_tracker_node.cpp`
  - Circle radius (1-4 pixels) based on optical flow velocity magnitude
  - Velocity magnitude as proxy for depth (closer features move faster)
  - Mapping: velocity / 10.0 → radius increment
  - Helps diagnose:
    - Scale/depth estimation issues
    - Feature motion patterns
    - Parallax distribution
  - Color still indicates track length (blue=new, red=long-tracked)

### Modified - Configuration Files

#### USB Camera Configuration
- **Updated** `config/usb_cam_config.yaml`
  - Added `use_bidirectional_flow` parameter
  - Added ZUPT parameters (vel_threshold: 0.05, acc_threshold: 0.1)
  - Changed `estimate_td` from 0 to 1 for time offset estimation

#### RVIZ Configuration
- **Updated** `config/vins_rviz_config.rviz`
  - Enabled `history_point` cloud visualization
  - Adjusted point cloud size and rendering
  - Modified camera view distance and angle
  - Updated window geometry

### Performance Improvements

#### Memory Efficiency
- **Grid allocation**: Replaced full `vector` with `unordered_map` for sparse grid storage
- **CLAHE reuse**: Single object creation instead of per-frame allocation
- **Sub-pixel refinement**: Limited to top-50 features instead of all detected

#### Processing Speed
- **Optional bidirectional flow**: Can be disabled for faster tracking
- **Grid-based selection**: Faster than sorting all features by response
- **AGAST detector**: Faster than Shi-Tomasi for equivalent quality

### Debugging & Diagnostics

#### Feature Visualization
- Circle size indicates estimated distance/depth
- Color indicates tracking age
- Helps identify:
  - Scale drift
  - Initialization problems  
  - Feature quality distribution

#### Console Output
- ZUPT status messages when velocity reset occurs
- CLAHE processing time tracking
- Feature detection timing breakdown

### Technical Details

#### File Changes
- `feature_tracker/src/feature_tracker.cpp`: Core detection and tracking logic
- `feature_tracker/src/feature_tracker.h`: Added CLAHE member variable
- `feature_tracker/src/feature_tracker_node.cpp`: Visualization modifications
- `feature_tracker/src/parameters.h/cpp`: Added FAST_THRESHOLD, USE_BIDIRECTIONAL_FLOW
- `vins_estimator/src/estimator.cpp`: ZUPT logic in processIMU()
- `vins_estimator/src/parameters.h/cpp`: Added ZUPT parameters
- `config/termal_cam_config.yaml`: Thermal camera complete configuration
- `config/usb_cam_config.yaml`: Parameter updates
- `vins_estimator/launch/termal_cam.launch`: New launch file

#### Commit History
- `37418d2`: Bidirectional optical flow implementation
- `6710bdb`: FAST corner detector (replaced by AGAST later)
- `a51f8bd`: AGAST feature detection
- `a7fdcab`: AGAST + Grid selection + Lucas-Kanade optimization
- `9024f49`: Thermal camera configuration

### Dependencies
- OpenCV 4.x (required for AGAST, CLAHE APIs)
- ROS (Kinetic/Melodic/Noetic)
- Eigen 3
- Ceres Solver

### Configuration Parameters Reference

#### Feature Tracker Parameters
```yaml
max_cnt: 150-180          # Maximum number of tracked features
min_dist: 10-30           # Minimum distance between features (pixels)
fast_threshold: 20-40     # AGAST corner detection threshold
use_bidirectional_flow: 0/1  # Enable forward-backward tracking
F_threshold: 1.0-2.0      # RANSAC threshold for fundamental matrix
equalize: 0/1             # Enable CLAHE histogram equalization
```

#### ZUPT Parameters
```yaml
enable_zupt: 0/1                 # Enable/disable zero velocity update
zupt_vel_threshold: 0.05-0.2     # Velocity threshold (m/s)
zupt_acc_threshold: 0.1-0.2      # Acceleration deviation threshold (m/s²)
```

### Known Issues & Future Work

#### Pending Optimizations
- Multi-threading for AGAST detection (parallel processing)
- Memory management improvements (unique_ptr migration in estimator.cpp)

#### System Stability
- Camera "flying away" issue under investigation
- Depth-based visualization added for diagnosis
- Potential causes:
  - Initialization quality (requires sufficient motion)
  - IMU calibration accuracy
  - Scale observability in monocular VIO

### IMU Requirements

**Supported IMU Types:**
- **6-DOF IMU required** (3-axis accelerometer + 3-axis gyroscope)
- Magnetometer NOT used (9-DOF sensors work but mag data ignored)
- Standard ROS `sensor_msgs/Imu` message format
- Required calibration parameters:
  - Measurement noise: `acc_n`, `gyr_n`
  - Bias random walk: `acc_w`, `gyr_w`

### References & Credits

- Original VINS-Mono: HKUST Aerial Robotics Group
- AGAST detector: Elmar Mair et al.
- CLAHE: Karel Zuiderveld
- Kalibr calibration toolkit: ASL ETH Zurich
- Branch: `UAV_perfom`
- Author: Devitt Dmitry

---

**Note**: This changelog tracks improvements made to the UAV performance optimization branch. For production deployment, thorough testing on target hardware is recommended.
