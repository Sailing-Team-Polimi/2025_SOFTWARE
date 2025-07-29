# 🛠️ CAN Bus Communication and Parsing – ROS 2 Package (`can_bus`)

This package provides a ROS 2 interface for communicating with a custom microcontroller via UART and parsing the received CAN bus data, including potentiometer and IMU values.

## 📦 Nodes Overview

### 1. `brunilde_simple_node`

Manages low-level UART communication with the external microcontroller using a custom **COBS encoding/decoding** mechanism via the `BusInterface` library.

- Publishes bus data on the topic `/fromBrunilde` using a custom message `SailMsgs::SerialMsg`.
- Subscribes to `/toBrunilde` to send commands back to the bus.
- Message type `SerialMsg` includes:
  - `uint8 id`: CAN ID
  - `uint8[] payload`: CAN payload

### 2. `general_parser_node`

A generic parser node that filters incoming messages from the bus by their CAN ID and converts the payload into `int64` values.

- Subscribes to `/raw_data` (usually remapped to `/fromBrunilde`).
- Publishes a single `std_msgs/Float64` message on `/parsed_data` containing the converted value.
- Parameters:
  - `id`: CAN ID to filter
  - `length`: expected payload length (in bytes)
  - `scale`: optional scale factor (default: 100)

### 3. `imu_parser_node`

Parses and synchronizes IMU data packets received over the bus. Once all required data packets (roll-pitch-yaw, velocity, acceleration) are received, it publishes:

- `sensor_msgs/Imu` on `/imu_data`
- Optionally, `geometry_msgs/PoseWithCovarianceStamped` on `/imu_pose`
- Optionally, `geometry_msgs/Vector3` on `/imu_ypr`

Also supports:
- Publishing static transforms from `base_link` to the IMU frame
- Customizable covariance matrices

## 🧰 Library

### `BusInterface`

Custom C++ library implementing:

- UART communication
- COBS encoding/decoding
- Message structuring (via `CanPackage`)
- Singleton-style `SerialInterface` access pattern

## 🚀 Launch Files

### `brunhild_and_parsers.launch.py`

Launches the full runtime setup:

- `brunilde_simple_node`
- `general_parser_node` (for wand potentiometer)
- `imu_parser_node`

Used when connecting to the live CAN bus system via UART.

### `pot_imu_parsers.launch.py`

Launches only the parser nodes:

- `general_parser_node`
- `imu_parser_node`

Used for post-processing of recordings or logs from a previous run.

## 📄 Message Definition

### `SailMsgs/SerialMsg.msg`

```text
builtin_interfaces/Time stamp
uint8 id
std_msgs/UInt8MultiArray payload
```

## 🧪 Example Usage

### Live Communication

```bash
ros2 launch can_bus brunhild_and_parsers.launch.py
```

### Parsing only

```bash
ros2 launch can_bus pot_imu_parsers.launch.py

```

## 🧠 Notes
- The BusInterface library abstracts away low-level UART and COBS logic, keeping node logic clean.
- All nodes are modular and reusable.
- Covariance values and TFs for IMU can be set via YAML config files.