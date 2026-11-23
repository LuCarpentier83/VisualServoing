# Visual Servoing — Autonomous BlueROV2

ROS 2 package that makes a **BlueROV2** underwater robot find and follow a buoy with its
camera. A vision node detects the buoy, either by HSV color tracking or with a custom-trained
**YOLOv5 Nano** model. A control node computes an image-based visual servoing law and drives
the thrusters through MAVROS. A state machine (searching, tracking, lost) keeps the robot
working when the buoy leaves the field of view.

## Architecture

```
camera ──► video ──► image_processing_tracker (HSV)   ──┐  tracked_point
                     image_processing_yolo   (YOLOv5) ──┤  tracked_segment
                                                        │  desired_point
                                                        ▼
joystick ──► joy ──────────────────────────────► listenerVS ──► rc/override ──► MAVROS ──► BlueROV2
IMU, depth, sonar (ping1d) ─────────────────────►  (visual servoing + state machine)
```

| Node (executable) | File | Role |
|---|---|---|
| `video` | `video.py` | Streams the camera image to ROS |
| `image_processing_tracker` | `image_processing_tracker.py` | HSV color tracking: center of the buoy and its width in meters |
| `image_processing_yolo` | `image_processing_yolo.py` | Same outputs from YOLOv5 detections |
| `listenerVS` | `listener_visual_servoing.py` | Visual servoing control, state machine, joystick, arming and modes |
| `listenerMIR` | `listenerMIR.py` | Manual teleoperation with stabilization (joystick / `cmd_vel`) |
| — | `pinger_node.py` | Ping1D sonar reader |

## How it works

**Vision.** In HSV mode, the image is converted to HSV and filtered around a color picked by
clicking on the image. The largest contour gives the buoy's center and its left and right
edges, converted to meters with the camera calibration. Detections are rejected when the
buoy is narrower than 0.07 m or when the point moves faster than 1 m/s between frames.
In YOLO mode, the center and edges come from the bounding box of the detected buoy. The
desired point is the image center by default and can be moved with a right click.

**Control.** The error between the desired and tracked points goes through the
pseudo-inverse of the interaction matrix to give the camera velocity. A homogeneous
transform turns it into a robot velocity, PID gains tune the response, and the result is
sent as PWM values (1100–1900) on `rc/override`. Forward motion stops once the buoy is
wide enough in the image, meaning it is close.

**State machine.**
- `SEARCHING`: spiral or figure-eight motion until the buoy is detected
- `TRACKING`: visual servoing on the buoy
- `LOST`: after a timeout without a detection, turn back toward the last known position,
  then search again

**Joystick.** Start / Back arm and disarm the motors, A / X switch between manual and
automatic modes, and other buttons tilt the camera and change the light intensity.

## Main topics

| Topic | Type | Direction |
|---|---|---|
| `/bluerov2/tracked_point`, `/bluerov2/desired_point`, `tracked_segment` | `Float64MultiArray` | vision → control |
| `rc/override` | `mavros_msgs/OverrideRCIn` | control → MAVROS |
| `joy`, `cmd_vel` | `Joy`, `Twist` | operator → control |
| `imu/data`, `global_position/rel_alt`, `ping1d/data` | `Imu`, `Float64`, `Float64MultiArray` | sensors → control |
| `visual_servoing/error`, `visual_servoing/cam_speed`, `visual_servoing/robot_speed` | `Float64MultiArray`, `Twist` | debug output |

## Requirements

- ROS 2 Iron (newer distributions should work)
- MAVROS (`mavros`, `mavros_msgs`), `joy`, `teleop_twist_joy`
- Python: `opencv-python`, `cv_bridge`, `numpy`, and `torch` plus
  [YOLOv5](https://github.com/ultralytics/yolov5) for the YOLO node

## Build and run

```bash
cd ~/ros2_ws        # this repository's src/ inside a ROS 2 workspace
colcon build
source install/setup.bash

ros2 launch autonomous_rov run_mavros.launch               # link to the ROV
ros2 launch autonomous_rov run_gamepad.launch              # joystick
ros2 launch autonomous_rov run_video.launch                # camera + HSV tracking
# or: ros2 launch autonomous_rov run_yolo_video.launch     # camera + YOLOv5
ros2 launch autonomous_rov run_listener_visualservo.launch # visual servoing + state machine
```

If the image does not display, check the conversion in the vision node: use
`imgmsg_to_cv2(msg, 'bgr8')` for `sensor_msgs/Image` or `compressed_imgmsg_to_cv2(msg)` for
`sensor_msgs/CompressedImage`.

## Training the YOLOv5 buoy detector

1. Put the images and their YOLO-format `.txt` labels in `data/images/train` and
   `data/images/valid`, and write a `data.yaml`:
   ```yaml
   train: <path>/data/images/train
   val: <path>/data/images/valid
   names:
     0: buoy
   ```
2. Train YOLOv5 Nano, which is fast enough for real time on the ROV:
   ```bash
   cd src/yolov5
   python3 train.py --img 640 --batch 16 --epochs 50 --data <path>/data.yaml --weights yolov5n.pt
   ```
3. The best weights are saved to `runs/train/exp*/weights/best.pt`. Point
   `image_processing_yolo.py` to that file. Training can be followed with
   `tensorboard --logdir runs/train`.

## Acknowledgements

- [YOLOv5](https://github.com/ultralytics/yolov5) by Ultralytics
- [ping-python](https://github.com/bluerobotics/ping-python) and the Ping sonar ROS driver by Blue Robotics

## License

Apache 2.0. See [LICENSE](LICENSE).
