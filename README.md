# Fast Frontier Detection (FFD) 

This repo contains Helm's Deep implementation of a FFD algorithm. Instructions for downloading and runing our code are described below. 

### Build the repo. 
1. `mkdir -p ~/ffd_ws/src`
2. `cd ~/ffd_ws/`
3. `catkin_make`
4. `cd src`
4. `git clone <this repository url>` (found in the upper right)
5. `cd ~/ffd_ws`
6. `catkin_make`

### Runing our demos. 

Our implementation of FFD was developed and tested on ROS Melodic. Execution on other ROS versions have not been tested and my not work as intended. For execution of our FFD algorithm please follow the steps below. 

Note that these instructions assume that the "desktop-full" version of ROS (which includes the Gazebo physics simulator) is installed. If a the "desktop" or "base" version are instead installed, additional effort may be needed to install Gazebo and maybe RVIZ.
