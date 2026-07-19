# tello_steel_defect_detection

![Demo Video](source/video/tello_defect_detection.mp4)

ROS 2 Jazzy pipeline for flying a DJI Tello EDU manually while detecting and annotating steel surface defects from the live camera stream.

## Project Scope
The goal of this project is to implement a real-time, aerial-to-desktop computer vision pipeline for automated steel surface inspection. The scope includes:

- Hardware Integration: Establishing a stable video telemetry link between a Tello EDU drone and a desktop host.

- Computer Vision Pipeline: Implementing a ROS 2-based system that captures, pre-filters, and crops drone imagery to isolate target defect regions on an inspection surface.

- Segmentation Inference: Executing a fine-tuned PyTorch model to perform semantic segmentation on captured defect regions, providing pixel-perfect localization of surface anomalies.

- Annotation/Visualization: Real-time generation of color-coded masks to visualize detected defects on a live video stream.

## Project Limitations
- Classification Accuracy: While the pipeline successfully achieves major defect localization and segmentation, it currently demonstrates insufficient performance in multi-class defect classification. The model struggles to reliably differentiate between specific defect sub-types of 4 classes.

- Domain Adaptation: The model was trained on high-resolution industrial sensor data; performance is degraded when inspecting printed test patterns on physical walls due to the inherent loss of contrast and the presence of paper texture, glare, and compression artifacts.

- Motion Blur and Drift: In-flight performance is constrained by the Tello EDU's optical flow stability. Featureless backgrounds or rapid drone movement may result in motion blur, limiting the resolution of captured defect imagery.

- Network Latency: The system relies on a 2.4GHz Wi-Fi link. Real-time inference performance is bound by network-induced packet loss and the inherent latency of the UDP-based H.264 video feed.

- Lighting Sensitivity: The segmentation pipeline relies on the contrast difference between the inspection target and the mounting wall. Variable ambient lighting significantly impacts detection stability.

## System Architecture
![System Architecture](source/images/tello_defect_detection_system_architecture.png)

## Workspace Setup

Run from the workspace directory:

```bash
cd ~/tello_steel_defect_detection/tello_ws
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONPATH="$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH"
```

Build after code or dependency changes:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Everything

Open the full pipeline in four terminals:

```bash
cd ~/tello_steel_defect_detection
./scripts/run_pipeline.sh
```

The launcher starts:

```text
tello_bridge_node
defect_detector_node
rqt_image_view /defect_detections/image
tello_keyboard_controller_node
```

The default model path is relative to the repo:

```text
tello_ws/src/tello_defect_pipeline/models/model.pth
```

Optional local overrides can go in `.env`:

```bash
cp .env.example .env
```

Then edit values such as `MODEL_PATH` or `MOVEMENT_TIMEOUT`.

## Run The Pipeline

Use separate terminals for each long-running command. Source the workspace setup in every terminal.

Terminal 1, publish Tello camera frames and listen for `/cmd_vel`:

```bash
ros2 run tello_defect_pipeline tello_bridge_node
```

Terminal 2, run defect detection on the camera stream:

```bash
ros2 run tello_defect_pipeline defect_detector_node --ros-args -p model_path:=/home/blancjh/tello_steel_defect_detection/tello_ws/src/tello_defect_pipeline/models/model.pth
```

Terminal 3, view the annotated output:

```bash
ros2 run rqt_image_view rqt_image_view /defect_detections/image
```

Useful checks:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /defect_detections/image
ros2 topic echo /cmd_vel
```

## Custom Keyboard Teleoperation

Run the project controller in its own focused terminal:

```bash
ros2 run tello_defect_pipeline tello_keyboard_controller_node
```

Controls:

```text
u : takeoff / hover
j : land
w : up (+z)
s : down (-z)
a : yaw left
d : yaw right
8 : pitch forward
5 : pitch backward
4 : roll left
6 : roll right
q : quit controller
```

Use the number row or numpad digits for `8`, `5`, `4`, and `6`. If the numpad does not respond, turn Num Lock on.

Movement keys are active only while held. When you release a movement key, terminal key repeat stops and the controller automatically returns to hover after a short timeout. Press `u` to take off into hover, press `j` to land, or press `q` to quit. The keyboard controller terminal must stay focused.

To verify that keyboard commands are reaching ROS:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /tello/takeoff
ros2 topic echo /tello/land
```

The Tello bridge converts `/cmd_vel` `geometry_msgs/Twist` messages into Tello RC commands and listens for `/tello/takeoff` and `/tello/land`.

## Standard Keyboard Teleoperation

The standard ROS teleop tool is still available if needed:

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

## Optional Gamepad Teleoperation

Install joystick support:

```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

Terminal 1, publish joystick input:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node
```

Terminal 2, convert joystick input to `/cmd_vel`:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_joy teleop_node --ros-args -r cmd_vel:=/cmd_vel
```

Check the joystick and velocity topics:

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

## Manual Scanning Workflow

1. Start `tello_bridge_node`.
2. Start `defect_detector_node`.
3. Start `rqt_image_view` on `/defect_detections/image`.
4. Start `tello_keyboard_controller_node` or gamepad teleop in a separate terminal.
5. Fly slowly across the printed steel surface images.
6. Watch the annotated output for target lock and defect segmentation.

## Safety Notes

Test command publishing before flying. For the first checks, keep the drone grounded, remove propellers if practical, or use a safe open area.

Keep an emergency stop plan ready. Press `j` to land, and `Ctrl-C` stops the teleop node after publishing zero velocity. If the drone is moving unexpectedly, stop sending movement commands and land using the Tello app or another known-good control method.

Avoid flying close to people, fragile objects, walls, or reflective surfaces. Move slowly while scanning so the detector has stable frames to process.
