# apriltag_docking

## install
```
mkdir -p autodock_ros2_ws/src
cd ~/autodock_ros2_ws/src
git clone https://github.com/H-HChen/apriltag_ros.git -b foxy-devel
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/H-HChen/apriltag_docking.git -b foxy-devel
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install    #ignore all warning plz 
``` 

## Modify tag family, tag size and tag_ids
```
cd ~/autodock_ros2_ws/
vim src/apriltag_ros/apriltag_ros/launch/tag_realsense.launch.py
vim src/apriltag_ros/apriltag_ros/config/setting.yaml
```

### tag_gazebo.launch.py
Set tags size and tag family
```
param = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.08,
    "max_hamming": 0,
    "threads": 4,
    "z_up": True
}

```
### setting.yaml

Remember to turn publish_tf on
```
tag_family:        'tag36h11' # options: tagStandard52h13, tagStandard41h12, tag36h11, tag25h9, tag16h5, tagCustom48h12, tagCircle21h7, tagCircle49h12
tag_threads:       4          # default: 2
tag_decimate:      1.0        # default: 1.0
tag_blur:          0.0        # default: 0.0
tag_refine_edges:  1          # default: 1
tag_debug:         0          # default: 0
# Other parameters
publish_tf:        true       # default: false

```
## simulation in gazebo
1. Launch Neuronbot2 and tag in gazebo
```
roslaunch neuronbot2_gazebo neuronbot2_world.launch world_model:=tag.model
```
2. Launch apriltag_docking 

    Remember to change names of camera_namespace and topic name
```
roslaunch auto_dock auto_dock.launch  camera_name:=/camera image_topic:=/raw_image camera_frame:=camera_link
```
