# nodemcu-ros-car
This code is intended to control a differential drive robot via ROS Twist messages using a NodeMcu or ESP8266. The car will respond to __Twist.linear.x__ and __Twist.angular.z__ values. If you have questions or improvements email me at reinhard.sprung@gmail.com
## Parts needed:
- Smart car chassis with at least 1 motor on each side, like this: [Smart car chassis 4WD](https://www.aliexpress.com/item/Smart-car-chassis-4WD-4-wheel-drive-force-the-chronological-magnetic-motor-With-code-disc-tachometer/32622219972.html?spm=a2g0s.9042311.0.0.27424c4djmBIqw)
- [NodeMCU](https://en.wikipedia.org/wiki/NodeMCU) (I'm using version 1.0)
- L298N motor driver like this: [L298N](https://www.aliexpress.com/item/L298N-Module-Dual-H-Bridge-Stepper-Motor-Driver-Board-Modules-for-Arduino-Smart-Car-FZ0407-Free/1761850243.html)
- Battery pack or other energy source. (I use 8x 1.2 Volt rechargable batteries)
- Cables
- Wifi
- Linux workstation to run ROS
- Gamepad (optional)

## Software:
- Arduino IDE
- Robot Operating System [http://www.ros.org/](http://www.ros.org/)

## Connections:
- Battery (+) to L298N +12V 
- Battery (-) to L298N GND
- L298N GND to NodeMcu GND
- L298N +5V to NodeMcu Vin
- Remove jumpers from L298N ENA and ENB
- L298N ENA to NodeMcu D1
- L298N IN1 to NodeMcu D2
- L298N IN2 to NodeMcu D3
- L298N IN3 to NodeMcu D5
- L298N IN4 to NodeMcu D6
- L298N ENB to NodeMcu D7
- L298N OUT1&2 to left motor (connect in parallel if >1)
- L298N OUT3&4 to right motor (connect in parallel if >1)

## Arduino IDE:
- Go to "Sketch/Include Library/Manage Libraries...", search for "ESP8266" and install that library
- Use the same method to install library "rosserial" or follow this tutorial: [Arduino IDE Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
- Insert your own Wifi SSID and password into [nodemcu-ros-car.ino](https://github.com/Reinbert/nodemcu-ros-car/blob/master/nodemcu-ros-car.ino)
- If you are using a different micro controller than the NodeMcu v1.0 find out what its highest possible PWM value is and change 'MAX_PWM' and 'MIN_PWN' accordingly. (Max PWM on Arduino is 255, while on a NodeMcu it's 1023)
- Flash [nodemcu-ros-car.ino](https://github.com/Reinbert/nodemcu-ros-car/blob/master/nodemcu-ros-car.ino) onto the NodeMcu

## ROS Installation:
- Install and configure ROS if you haven't done so yet. I'm using Kinetic Kame on Ubuntu 16.04. [Installation Tutorial](http://wiki.ros.org/ROS/Installation)
- Install [rosserial_server](http://wiki.ros.org/rosserial_server) via terminal: `sudo apt-get install ros-kinetic-rosserial-server`
- Install [teleop_twist_joy](http://wiki.ros.org/teleop_twist_joy) if you have a gamepad: `sudo apt-get install ros-kinetic-teleop-twist-joy` or
- Use the keyboard teleop [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) `sudo apt-get install ros-kinetic-teleop-twist-keyboard`
## ROS Operation:
- Launch rosserial socket node: `roslaunch rosserial_server socket.launch`
- Launch gamepad node in a new terminal: `roslaunch teleop_twist_joy teleop.launch joy_config:="insert gamepad type"` or 
- Keybord node: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

This should suffice to make the robot car connect to your Wifi and rosserial server and follow twist commands from your gamepad.
## Alternative Teleop:
I've written this small ROS node ([teleop_joy_race](https://github.com/Reinbert/teleop_joy_race)) which allows to control this robot like a car in a racing video game. Pressing __Right Trigger__ will accelerate the car, while pressing __Left Trigger__ will decelerate and make the robot go backwards. Steering is done via the __Left Stick__. In the default configuration you are required to hold down the deadman button (button A on xbox360) while controlling for safety reasons. The node should work with any joystick that is supported by Linux. However, I only tested it with an Xbox360 wireless controller and therefore only provided a config file for that controller. Feel free to create and send me yours.
#### Installation: 
```sh
cd ~/catkin_ws/src
git clone https://github.com/Reinbert/teleop_joy_race
cd ..
catkin_make
source devel/setup.bash
```
#### Launch:
```sh
roslaunch teleop_joy_race teleop.launch
```
Launch teleop AND a rosserial socket at the same time. You don't need to call `roslaunch rosserial_server socket.launch` anymore:
```sh
roslaunch teleop_joy_race teleop_serial.launch 
```
Disable deadman button. USE AT YOUR OWN RISK!
```sh
roslaunch teleop_joy_race teleop.launch enable_deadman:=false 
roslaunch teleop_joy_race teleop_serial.launch enable_deadman:=false
```
## Troubleshooting:
- Open Serial Monitor in Arduino IDE and check for Wifi and ROS connection output. 
- The LED on the NodeMcu should turn on (only) if its connected to both Wifi and ROS.
- [teleop_twist_joy](http://wiki.ros.org/teleop_twist_joy) uses an enable (deadman) button which needs to be pressed while using the gamepad. Find out which one in the [config](https://github.com/ros-teleop/teleop_twist_joy/tree/indigo-devel/config).
- Test gamepad with `jstest /dev/input/js0`
- Make sure the teleop node is outputting commands: `rostopic echo /cmd_vel`
