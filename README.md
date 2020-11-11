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
set tags id you want
```
standalone_tags:
  [
    {id: 0, size: 0.08 , frame_name: tag_0},
    {id: 10, size: 0.03}
  ]

```
