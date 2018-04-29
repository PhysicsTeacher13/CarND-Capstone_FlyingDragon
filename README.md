# Self Driving Car Nanodegree Capstone project

## Introduction

This read me forms the basis of the submission report for team FlyingDragons.

The team consists of the following team members

| Name | Email address |
| ------------- |:-------------:|
| Wilfred Henry| physicsteacher13@gmail.com  |
| Lybron Sobers|	lybronsobers@icloud.com|
| Mostafa Mansour | mostafa.e.mansour@gmail.com|
| Jose Antonio de la Fuente| delafu@gmail.com|
| George Haigh|	georgehaigh@hotmail.com |
| Fabrizio Frigeni| ffrige@hotmail.com   |


### Waypoint Updater

TODO

### DBW Node

TODO

### Traffic Light Detector

The traffic light detector was based on the starter code from the project walkthrough.

###### A summary of the functionality follows:

The coordinates of the Traffic Lights are received by subscribing to the '/vehicle/traffic_lights' topic. This also includes the status of each light which was used in order to test the rest of the system.
The coordinates of the stop lines is returned from the yaml configuration file.

As the cooridinates of the pose of the lights is static, they are only read on the traffic_lights topic to try to limit the latency that was experienced during development.

A KDTree of the base waypoints is created by subscribing to the "base_waypoints" message.
Once the coordinates of the traffic lights and stop lines are known, the nearest waypoint to these is obtained by querying the KDTree containing the base waypoints.
Again these activities are only done once to minimised latency.

The vehicle pose is received on the /current_pose topic.

A loop is run at a process the traffic lights.
This involves finding the nearest waypoint to the cars position, and finding the closest stop line and traffic light in from of the vehicle, by using the waypoint indicies of these to judge the distance.

The state and index of the closest traffic light is then published on the '/traffic_waypoint' topic

### Traffic Light Detector

#TODO


### Project Issues

The team experienced issues with the Udacity Simulator during this project. Enabling the camera results in significant lag, which causes the PID to be unable to maintain control of the steering.
This appears to be a known issue, this discussions on slack in online.
(https://github.com/udacity/sdc-issue-reports/issues/1217)

The following attempts were made to recify this:
- Running the simulator on an additional PC, and connecting via ports
- Using the minimal graphic quality settings
- Modifying the server.py
- Adjusting the VM memory and CPU setting to give more processing power to the VM, and alternately the host
- Modifying the tcp buffer size using sysctl.configuration
- Unsubscribing the the image_color topic when not near a traffic light

None of these were particularly useful, the only real solution was to disable the camera when not in use (i.e when between traffic signs)

## Reference

For completeness, the original Readme follows below this point for refernce purposes.


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
