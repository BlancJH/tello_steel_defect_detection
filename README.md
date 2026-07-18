# tello_steel_defect_detection

## Run Instruction

```bash
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONPATH="$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH"
ros2 run tello_defect_pipeline tello_bridge_node
```

```bash
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONPATH="$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH"
ros2 run tello_defect_pipeline defect_detector_node --ros-args -p model_path:=/home/blancjh/tello_steel_defect_detection/tello_ws/models/model.pth
```

```bash
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONPATH="$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH"
ros2 run rqt_image_view rqt_image_view /defect_detections/image
```