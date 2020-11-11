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
## Quickstart

### step1 Modify tag.yaml and setting.yaml
```
cd ~/autodock_ros1_ws/
vim src/
vim 
```
