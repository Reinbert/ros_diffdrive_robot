/*

  Arduino code to control a differential drive robot via ROS Twist messages using a NodeMcu or ESP8266
  If you have questions or improvements email me at reinhard.sprung@gmail.com

  Launch a ros serial server to connect to:
    roslaunch rosserial_server socket.launch

  Launch a teleop gamepad node:
    roslaunch teleop_twist_joy teleop.launch joy_config:="insert your gamepad"


  MIT License

  Copyright (c) 2018 Reinhard Sprung

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#include <math.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define LED_BUILTIN 2 // Remapping the built-in LED since the NodeMcu apparently uses a different one.
#define LED_BUILTIN_RED 16
#define MAX_PWM 1023  // NOTE: Max PWM value on a NodeMcu is 1023, while on Arduino it's only 255.
#define MIN_PWM 300   // The min amount of PWM the motors need to move. Depends on the battery, motors and controller.

// Pins
const uint8_t L_PWM = D1;
const uint8_t L_BACK = D2;
const uint8_t L_FORW = D3;
const uint8_t R_BACK = D5;
const uint8_t R_FORW = D6;
const uint8_t R_PWM = D7;

const char* ssid     = "----SSID----";
const char* password = "--PASSWORD--";

// Declare functions
void setupPins();
void setupSerial();
void setupWiFi();
void onTwist(const geometry_msgs::Twist &msg);

bool checkRosConnection(bool connected);
float mapPwm(float x, float out_min, float out_max);

// ROS
IPAddress server(192, 168, 0, 12);
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

bool _connected = false;

void setup()
{
  setupPins();
  setupSerial();
  setupWiFi();

  // Connect to rosserial socket server and init node. (Using default port of 11411)
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
}

void setupPins()
{
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(L_PWM, OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  stop();
}

void setupSerial()
{
  Serial.begin(115200);
  Serial.println();
}

void setupWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
}

void stop()
{
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void onTwist(const geometry_msgs::Twist &msg)
{
  if (!_connected)
  {
    stop();
    return;
  }

  /*
    Serial.print(msg.linear.x);
    Serial.print("|");
    Serial.print(msg.linear.y);
    Serial.print("|");
    Serial.print(msg.linear.z);
    Serial.print("   ");

    Serial.print(msg.angular.x);
    Serial.print("|");
    Serial.print(msg.angular.y);
    Serial.print("|");
    Serial.print(msg.angular.z);
    Serial.print("   ");
  */

  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. MAX_PWM = full speed, while MIN_PWM = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), MIN_PWM, MAX_PWM);
  uint16_t rPwm = mapPwm(fabs(r), MIN_PWM, MAX_PWM);

  /*
    Serial.print(l);
    Serial.print("|");
    Serial.print(r);
    Serial.print("   ");

    Serial.print(lPwm);
    Serial.print(" | ");
    Serial.println(rPwm);
  */

  // Set direction pins and PWM
  digitalWrite(L_FORW, l > 0);
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
}

void loop()
{
  if (!rosConnected())
    stop();
  node.spinOnce();
}


bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "Connected" : "Disconnected");
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

