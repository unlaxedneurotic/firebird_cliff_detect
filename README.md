# Cliff and Object Detection - Fire Bird VI
This project uses the 8 inbuilt ultrasonic sensors and one additional ultrasonic sensor interfaced via Arduino Uno [Arduino Ultrasoninc Sensor HC-SR04] to detect objects and cliffs. The HC-SR04 sensor should be attached to the front of the robot via an extended rod and facing vertically down towards the floor. The setup looks something like the following diagrams.

![side_view_diagram](/diagram/cliff_detect_side_view.png) 

#### Our test setup
![setup_diagram](/diagram/setup_picture.jpg)

## List of packages
A list of packages included in this repository - 
1. `cliff_detect` - ROS package which implements the cliff and object detection. Receives movement commands and sensor data and takes decision to move the robot
2. `arduino_sketch` - contains an arduino sketch which calculates the distace and publishes it as a ROS topic on the host robot.
3. `mqtt_conv` - recieves movement commands in json from an mqtt server and publishes it as a `Twist` message. The corresponding android application that can send movement commands to an mqtt server can be found here(https://github.com/unlaxedneurotic/bitsrobocontrollerv2)

#### A bird's eye view
![process_flow](/diagram/process_flow.png)

### `cliff_detect`
Contains the `cliff_detect_run.py` script.
This script will subscribe to the following topics:
1. `arduino_dist` - data from arduino ultrasonic sensor that measures dist to the floor
2. `ros0xrobot/sonar` - data from the firebird inbuilt sonars
3. `ros0xrobot/mqtt_vel` - data from mqtt server that will be served by mqtt_conv

The sonar sensors are indexed as follows: view from top - values start from zero with the leftmost sensor and follows clockwise.
After checking for obstacles or cliffs in the direction of the robot's movement, we take decision on whether we wish to let the robot move or not, or in the case it is moving, to stop it. The `Twist` message is published to the topic `ros0xrobot/cmd_vel`.
When the script is first launched, it calibrates the distance to the floor using the first 5 readings. This is to allow the user to place the sensor in any position as long as it as it has a direct view of the floor. 

**A detailed diagram of how it all works:**
![topic_flow](/diagram/topic_schema.png)

### `arduino_sketch`
Contains the sketch `ultrasonic_dist_measure` which needs to be uploaded onto an arduino uno. It measures the distance in meters to the floor as well as provides an interface to send the data to ROS. It itself acts as a ROS node and using `rosserial`, publishes the measured data to a ros topic. Currently, it publishes a `float` to the topic `arduino_dist`.
We also need to run a rosserial node on the host robot to deserialize the data that is sent via the arduino.

### mqtt_conv
Contains the script `talker.py`
It is an mqtt client that connects to an mqtt server and subscribes to the specified topic on which we send the movement commands via the android application or any other mqtt publisher. The data that is published on the server must be in json in the following format:
```json
{
    "twist_message":{
        "linear":{
            "x":0.2,
            "y":0.0,
            "z":0.0
        },
        "angular":{
            "x":0.0,
            "y":0.0,
            "z":0.2
        }
    }
}
```
The received json data will be converted into a `Twist` message and published onto the topic of your choice. Currently it is being published onto `/ros0xrobot/mqtt_vel`

## Installation and Dependencies
We assume that we have a working ROS installation. For each ROS package, you need to create a package in your ROS workspace, copy the scripts folder there and then run `catkin_make`. 

To run `mqtt_conv`, ensure to install the paho-mqtt client using `pip install paho-mqtt`

#### Setting up HC-SR04 with Arduino Uno
You can install the Arduino IDE and `rosserial` libraries from the following [link](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). If you have worked with arduino and have the IDE already installed, just install `rosserial` using:
```
sudo apt-get install ros-<ros-distribution>-rosserial-arduino
sudo apt-get install ros-<ros-distribution>-rosserial
``` 

Follow this schema to connect the sensor to the arduino:
![schema](/diagram/arduino_schema.jpg)

Upload the sketch to your arduino board using this [link](https://www.arduino.cc/en/Tutorial/getting-started-with-ide-v2/ide-v2-uploading-a-sketch).
*Note: Ensure to close all rosserial nodes before uploading a sketch to the arduino board* 
## Running
Follow these steps to run the project:
1. Start `roscore`
2. Start the `rosserial` node to publish serialized data coming from the arduino to a `rostopic`
3. Start the robot's sensors
4. Start `mqqt_conv` node to get movement commands
5. Finally start the `cliff_detect` node

```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
roslaunch ros0xrobot ros0xrobot_sonar.launch
rosrun mqtt_conv talker.py
rosrun cliff_detect cliff_detect_run.py
```