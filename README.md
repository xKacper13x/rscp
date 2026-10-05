# RSCP – Rover Serial Communication Protocol Bridge

A ROS node that bridges the rover's autonomy stack with the base station over a serial link.
Commands from the base station are decoded and published to ROS topics; rover telemetry and
autonomy events are serialized and sent back.

Developed as part of the **SKA Robotics** (Student Space Association, Warsaw University of Technology)
software team for the **Anatolian Rover Challenge 2026**.

> **Status:** The team ultimately did not compete at ARC 2026, so the bridge has not been field-deployed.
> It was developed and tested end-to-end on virtual serial ports (`socat`) with a mock base station.

## How it works

```
Base station  <──serial──>  RscpTransceiver  <──callback──>  RscpRosBridge  <──topics──>  ROS autonomy stack
              COBS + Protobuf   (pure Python,                     (ROS node)
                                no ROS dependency)
```

- **`RscpTransceiver`** handles the wire protocol only: framing, (de)serialization and the serial port.
  It runs a background reader thread and passes decoded requests to a registered callback.
  It has no ROS dependency, so it can be reused or tested on its own.
- **`RscpRosBridge`** is the ROS node. It translates incoming requests into `AutonomyCommand` messages,
  listens to telemetry and autonomy events, and sends a `RoverStatus` heartbeat at 1 Hz.

### Wire protocol

- Messages are defined in [`rscp.proto`](rscp.proto) as two envelopes using `oneof`:
  `RequestEnvelope` (base station → rover) and `ResponseEnvelope` (rover → base station).
- Each serialized message is **COBS-encoded** and terminated with a `0x00` byte, which gives
  unambiguous frame boundaries on a raw byte stream. Frames longer than 1024 bytes are discarded.
- Malformed frames (COBS or Protobuf decode errors) are logged and skipped without stopping the reader.

**Requests (base station → rover):** `arm_disarm`, `set_stage`, `navigate_to_gps`, `search_area`, `start_exploration`.
Every recognized request is answered with `Acknowledge`.

**Responses (rover → base station):** `Acknowledge`, `TaskFinished`, `GPSCoordinate`, `RoverStatus`, `distance`.

### ROS interface

| Topic                | Type                    | Direction | Purpose                                     |
|----------------------|-------------------------|-----------|---------------------------------------------|
| `/autonomy/commands` | `rscp_bridge/AutonomyCommand` | publish   | Commands received from the base station     |
| `/autonomy/events`   | `rscp_bridge/AutonomyEvent`   | subscribe | Task finished / point found / measured distance |
| `/gps/fix`           | `sensor_msgs/NavSatFix` | subscribe | Current position for `RoverStatus`          |

| Parameter   | Default        | Description          |
|-------------|----------------|----------------------|
| `~port`     | `/dev/ttyUSB0` | Serial port          |
| `~baudrate` | `115200`       | Serial baud rate     |

## Repository structure

```
catkin_ws/src/   ROS 1 package (bridge node, transceiver, message definitions)
rscp.proto       Protocol definition
main.py          Mock base station used for local testing
```

## Running locally (without hardware)

Requires a ROS 1 environment (e.g. a Docker container with ROS installed).

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
2. Generate the Protobuf code (output into the package's `scripts/` directory):
   ```bash
   protoc -I=. --python_out=<path-to-scripts-dir> rscp.proto
   ```
3. Create a pair of linked virtual serial ports:
   ```bash
   socat -d -d pty,raw,echo=0 pty,raw,echo=0
   ```
   `socat` prints two device paths, e.g. `/dev/pts/3` and `/dev/pts/4`. One is used by the bridge,
   the other by the mock base station. Set the second one in `main.py`.
4. Start ROS and the bridge (separate terminals):
   ```bash
   roscore
   python3 -m scripts.rscp_ros_bridge _port:=/dev/pts/3
   ```
5. Start the mock base station:
   ```bash
   python3 main.py
   ```

**Expected result:** the bridge logs a received `navigate_to_gps` command and the mock base station
receives an `Acknowledge`. After about 3 seconds the mock reports the task result and its completion.

## Known limitations / TODO

- Rover state and heading in `RoverStatus` are not yet updated from ROS (always `DISARMED` / `0.0`).
- Battery telemetry subscriber is disabled until the CAN bus message type is finalized.
- Unknown commands are logged but not answered with an explicit error response.
- The generic string `message` response is not implemented.
- Unit tests for the transceiver (e.g. round-trip encoding over pyserial's `loop://` port) are planned.

## Tech stack

Python · ROS 1 (rospy) · Protocol Buffers · COBS · pyserial
