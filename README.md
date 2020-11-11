# apriltag_docking

## install
```
mkdir -p autodock_ros1_ws/src
cd ~/autodock_ros1_ws/src
git clone https://github.com/H-HChen/apriltag_ros.git
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/H-HChen/apriltag_docking.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
sudo pip3 install -U catkin_tools
catkin build   #ignore all warning plz 
``` 

## Modify tag.yaml and setting.yaml
```
cd ~/autodock_ros1_ws/
vim src/apriltag_ros/apriltag_ros/config/tag.yaml
vim src/apriltag_ros/apriltag_ros/config/setting.yaml
```

### tag.yaml
Set tags id ,size and frame name of tf
```
standalone_tags:
  [
    {id: 0, size: 0.08 , frame_name: tag_0},
    {id: 10, size: 0.03}
  ]

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
