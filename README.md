# tello_steel_defect_detection

## ROS Jazzy Python environment

ROS Jazzy cv_bridge is built against the NumPy 1.x ABI. Keep the workspace virtual environment on NumPy 1.26 and an OpenCV 4.x wheel:

```bash
cd tello_ws
source venv/bin/activate
python -m pip install --force-reinstall "numpy==1.26.4" "opencv-python==4.10.0.84"
python -m pip install -r requirements.txt
colcon build --symlink-install
```

After rebuilding, source ROS and the workspace before running nodes:

```bash
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONPATH="$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH"
ros2 run tello_defect_pipeline tello_bridge_node
```

```
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONPATH="$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH"
ros2 run tello_defect_pipeline tello_bridge_node
```
```
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONPATH="$PWD/venv/lib/python3.12/site-packages:$PYTHONPATH"
ros2 run tello_defect_pipeline defect_detector_node --ros-args -p model_path:=/tello_ws/models/model.pth
```