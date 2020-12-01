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
vim src/apriltag_docking/autodock_controller/param/neuronbot.yaml
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
### neuronbot.yaml

Set tag family and tad id
```
/autodock_controller:
  ros__parameters:
      cmd_vel_angular_rate: 0.25
      cmd_vel_linear_rate: 0.25
      default_turn: 1.0
      final_approach_distance: 1.0
      finish_distance: 0.5
      jog_distance: 0.2
      lost_tag_max: 5
      max_center_count: 10
      tune_angle: 0.42
      tag_id: "0"
      tag_family: "36h11" 
```
## simulation in gazebo
1. Launch Neuronbot2 and tag in gazebo
```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=tag.model use_camera:=top
```
2. Launch apriltag_docking 

    Remember to change names of camera_namespace and topic name
```
ros2 launch apriltag_docking autodock_gazebo.launch.py
```
