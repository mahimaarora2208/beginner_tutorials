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
- ROS2 Distro : ROS2 Humble
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` 
- ROS2 Humble Installation : [link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

## How to Run the ROS Package
### Build Instructions
```
cd <your_ROS2_ws>/src
git clone https://github.com/mahimaarora2208/beginner_tutorials.git
cd ..   
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select beginner_tutorials
source . install/setup.bash
source ~/<your ROS2 installation>/install/local_setup.bash
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
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```
