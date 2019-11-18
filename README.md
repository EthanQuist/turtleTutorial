# Roaming Turtlebot Tutorial

## Overview
This respository holds the code for a Turtlebot tutorial provided by ROS, WillowGarage, and Turtlebot. The code uses both the Turtlebot and Gazebo simulation environment to demonstrate ROS's capabilties in a simulated world. As the code runs the Turtlebot will roam around the simulated world and 

## Build
In order to build this project make sure to have a working catkin workspace. Inside the src folder of your catkin workspace you can download this repository. 
```
cd ~/catkin_ws/src
git clone https://github.com/EthanQuist/turtleTutorial.git
```
In order to build the package after it is downloaded use the following line of code:
```
cd ~/catkin_ws
catkin_make
```

## Run Simulation
Running this respository is straight forward because it uses a launch file. Running the launch file will start the Gazebo simulation as well as start the walker node which allows the Turtlebot to move. There is an option within the launch file to record a bagfile but the default choice is to not record. Follow the code below to run the simulation:
```
cd ~/catkin_ws/src/turtleTutorial/launch/
roslaunch myLaunch.launch
```


## Assumptions / Dependancies
This respository has many assumptions and dependancies to run. Most importantly is that this code is using ROS Kinetic. Make sure to have this version of ROS in order to run the code. There are also several other tutorials that need to be downloaded in order to run the code. To download the dependancy packages run the following code:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```


## Recording bagfiles
ROS has the ability to record all of the messages and topics being used during a package's run. The code to record a bag file is:
```
rosbag record -a
```
The launch file, however, already has built in functionality to record at launch. In order to record at launch the argument "record" must be true. This can be achieved by running the launch file making sure to include record:="true".

## Inspecting bagfiles
Bagfiles can be inspected by going into the results folder of the repository and then running the following code:
```
rosbag info "newbag.bag"
```


## Playback bagfiles
In order to playback a bag file it is important to note that Gazebo should not be running. Inside the results folder is where the rosbags are recorded. After navigating to that folder run the following code:
```
rosbag play "newbag.bag"
```




