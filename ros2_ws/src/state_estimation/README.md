# *state_estimation* – Dual-EKF Stack for the Foiling Moth  
**ROS 2 Humble – C++**

This package provides a complete **state estimation chain** for a foiling Moth sailboat. It is based on **two `robot_localization` EKFs** and several helper nodes that preprocess raw sensor data (GPS, IMU, wand potentiometer) into ROS messages compatible with the filters.

---

## 1️⃣ Core Nodes

### 🛰️ `gps_to_local`

| Feature           | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| Node (Executable) | `gps_to_local`                                                              |
| Published Topics | `/myodometry_gps`                                                           |
| Main Job         | Converts `NavSatFix` (lat/lon) + optional heading into `nav_msgs/Odometry` in the **map** frame. Subtracts the first GPS fix so coordinates start near (0,0). |

---

### 📏 `height_node`

| Feature           | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| Node (Executable) | `height_node`                                                               |
| Published Topics | `/height_stamped`                                                           |
| Parameters       | Inside `sensor_fusion.launch.py`                                            |
| Main Job         | Computes **ride height** (CM → water) from *wand angle + IMU roll/pitch*, publishes a `PoseWithCovarianceStamped`. |

**Height Formula:**
```cpp
height = (L_wand * std::sin(wand_rad) - L_rod * std::sin(pitch)) * std::cos(roll);
```

Where:
- `wand_rad` = wand angle in radians (from potentiometer)
- `pitch`, `roll` = angles from IMU orientation
- `L_wand` = wand length [m], parameter
- `L_rod` = horizontal offset from CM to wand pivot [m], parameter

> Only `pose.position.z` and `covariance[zz]` (variance in m²) are filled. All other covariance entries are set to 1e9 so that the EKF ignores them.

---

### 🌍 `local_to_gps`

| Feature           | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| Node (Executable) | `local_to_gps`                                                              |
| Published Topics | `/gps/filtered` (global)                                                    |
| Main Job         | Optionally reprojects local EKF output back to lat/lon for logging or dashboards. |

---

### 🧠 `ekf_node` (two instances)

| Feature           | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| Node (Executable) | `ekf_node` (run twice: global + local)                                      |
| Published Topics | `/odometry/filtered` (global), `/odometry/local` (local)                    |
| Parameters       | Defined in the `config` file                                                 |
| Main Job         | 
- **Global EKF** fuses `/myodometry_gps` (+ yaw) → computes TF **map → odom**  
- **Local EKF** fuses IMU, height, etc. → computes TF **odom → base_link**

---

## 2️⃣ Build & Run

> ⚠ **IMU data must be in ENU frame** (East-North-Up)

```bash
# Clone this package inside your ROS 2 Humble workspace

colcon build --packages-select state_estimation    # ⬅️ Build the package
source install/setup.bash                          # ⬅️ Source the workspace

# One-shot launch
ros2 launch state_estimation sensor_fusion.launch.py
```

---

### ⚓ Summary

**Build → Launch → Sail**

> Developed by Polimi Sailing Team  
> ✍️ Francesco Persio
