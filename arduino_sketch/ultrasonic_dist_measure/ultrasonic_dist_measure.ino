#include<ros.h>
#include<std_msgs/Float32.h>
// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Original source: Arbi Abdul Jabbaar
// Re-written by: Saransh Jindal
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// ---------------------------------------------------------------- //

/*
 * This sketch uses an ultrasonic sensor to measure distance of 
 * an obstacle in its path and publishes that data via ros-serial to 
 * a ros topic.
 * You also need to run a rosserial node on the host ros to de-serialize the incoming data
 * and actually publish the message on the ros topic
 */


#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement
ros::NodeHandle nh;
std_msgs::Float32 dist_msg;

ros::Publisher dist_pub("arduino_dist", &dist_msg);
// you can change the publisher topic to be anything
// just ensure that receiver endpoints on ROS host are also changed


void setup() {

  nh.initNode();
  nh.advertise(dist_pub);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
//  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
//  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
//  Serial.println("with Arduino UNO R3");
}
void loop() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  distance = distance>300?300:distance; // clipping the max distance to 3 meters
  dist_msg.data = distance/100;
  dist_pub.publish(&dist_msg);
  nh.spinOnce();
  // Displays the distance on the Serial Monitor
//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println(" cm");
  delay(200);
}
