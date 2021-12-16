# Cliff and Object Detection - Fire Bird VI
This project uses the 8 inbuilt ultrasonic sensors and one additional ultrasonic sensor interfaced via Arduino Uno [Arduino Ultrasoninc Sensor HC-SR04] to detect objects and cliffs. The HC-SR04 sensor should be attached to the front of the robot via an extended rod and facing vertically down towards the floor. The setup looks something like the following diagrams.

![side_view_diagram](/diagram/cliff_detect_side_view.png) 

#### Our test setup
![setup_diagram](/diagram/setup_picture.jpg)

## List of packages
A list of packages included in this repository - 
1. `cliff_detect` - ROS package which implements the cliff and object detection. Receives movement commands and sensor data and takes decision to move the robot
2. `arduino_sketch` - contains an arduino sketch which calculates the distace and publishes it as a ROS topic on the host robot.
3. `mqtt_conv` - recieves movement commands in json from an mqtt server and publishes it as a `twist` message. The corresponding android application that can send movement commands to an mqtt server can be found here(https://github.com/unlaxedneurotic/bitsrobocontrollerv2)

### `cliff_detect`
Contains the `cliff_detect_run.py` script.
This script will subscribe to the following topics:
1. `arduino_dist` - data from arduino ultrasonic sensor that measures dist to the floor
2. `ros0xrobot/sonar` - data from the firebird inbuilt sonars
3. `ros0xrobot/mqtt_vel` - data from mqtt server that will be served by mqtt_conv

The sonar sensors are indexed as follows: view from top - values start from zero with the leftmost sensor and follows clockwise.
After checking for obstacles or cliffs in the direction of the robot's movement, we take decision on whether we wish to let the robot move or not, or in the case it is moving, to stop it. The `Twist` message is published to the topic `ros0xrobot/cmd_vel`.
When the script is first launched, it calibrates the distance to the floor using the first 5 readings. This is to allow the user to place the sensor in any position as long as it as it has a direct view of the floor. 

A diagram that describes the entire process is shown next.

### `arduino_sketch`
Contains the sketch `ultrasonic_dist_measure` which needs to be flashed onto an arduino uno. It measures the distance in meters to the floor as well as provides an interface to send the data to ROS. It itself acts as a ROS node and using `rosserial`, publishes the measured data to a ros topic. Currently, it publishes a `float` to the topic `arduino_dist`.
We also need to run a rosserial node on the host robot to deserialize the data that is sent via the arduino.

### mqtt_conv
Contains the script `talker.py`
It is an mqtt client that connects to an mqtt server and subscribes to the specified topic on which we send the movement commands via the android application or any other mqtt publisher. The data that is published on the server must be in json in the following format:
```json
"twist_message": {
    "linear":{
        "x": 0.2,
        "y": 0.0,
        "z": 0.0
    },
    "angular":{
        "x": 0.0,
        "y": 0.0,
        "z": 0.2
    }
}
```
The received json data will be converted into a `Twist` message and published onto the topic of your choice. Currently it is being published onto `/ros0xrobot/mqtt_vel`