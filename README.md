[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Publisher and Subscriber in ROS2

### Overview

This is a ROS Package that defines the following:
- Basic publisher and subscriber with custom message.
- Use of all five logger levels: ```Fatal, Error, Warn, Info and Debug.```
- Custom service to change base output string.



### Dependencies/ Assumptions
- OS : Ubuntu 20.04 
- ROS2 Distro : ROS2 Galactic
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` 
- ROS2 Galactic Installation : [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

## How to Run the ROS Package
### Build Instructions
```
cd <your_ROS2_ws>/src
git clone https://github.com/mahimaarora2208/beginner_tutorials.git
cd ..   
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select beginner_tutorials
source . install/setup.bash
source ~/<your ROS2 installation>/opt/ros/galactic/setup.bash
```

### Run Publisher
To run the publisher node, open a new terminal and run:
```
cd <your_ROS2_ws>
. install/setup.bash
ros2 run beginner_tutorials talker
```
### Run Subscriber
To run the subscriber node, open a new terminal and run:
```
cd <your_ROS2_ws>
. install/setup.bash
ros2 run beginner_tutorials listener
```

### Update string using service
Run talker and Listener node by following above steps, Run
```
ros2 service call /update_request beginner_tutorials/srv/ChangeString "{input: 'myNewInput'}"
```
This will change the string on both nodes to "myNewString".


### Launch File
To launch nodes using launch file and pass parameters using command line:
```
cd <ROS2_ws>/
. install/setup.bash
 ros2 launch beginner_tutorials pub_sub.launch.yaml frequency:=5.0

```

To check whether the value updated to 5.0,open a new terminal and source it. Run:
```
ros2 param list
ros2 param get /minimal_publisher/sim frequency 
```
### tf2 Frames
To create pdf file of frames:
```
ros2 run tf2_tools view_frames
```
To check transformation between two frames (world and talk frames), run the publisher(talker) file and open a new terminal to run:
```
ros2 run tf2_ros tf2_echo world talk
```
### ROS2 Bags
To record rosbags in a custom folder:
```
ros2 bag record -o rosbag_files /chatter
```
To check topic information that is in the rosbag:
```
ros2 bag info rosbag_files
```
Lastly, to play back the data from rosbags:
```
ros2 bag play rosbag_files
```

The results for the listener displaying output based on rosbags is in the result folder. Please note, it works independent of publisher node.

### ROS2 Run TESTS
Run the following commands to test your test cases:
```
colcon build --packages-select beginner_tutorials
source install/setup.bash
colcon test beginner_tutorials
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials 
colcon test-result --test-result-base build/beginner_tutorials
echo $?
```
## Results
The results after running the following commands are stored in the <your_package>/results folder.

### rqt Console
```
 ros2 run rqt_console rqt_console

```
### cppcheck
Run the following command from the root directory of your ROS package
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint
Run the following command from the root directory of your ROS package
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```
### Google Styling format
Run the following command from the directory where the .cpp files are present(src in this case)
```
clang-format -style=Google -i your_file.cpp
```
