# Control software for differential drive smart car chassis using a NodeMcu, Arduino and the Robot Operating System (ROS)
This software is intended to control a 4 wheel differential drive robot via [ROS](https://www.ros.org/) messages using a NodeMcu or ESP8266. The software will respond to [geometry_msgs/Twist](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messages (precisely __Twist.linear.x__ and __Twist.angular.z__) and generate [PWM](https://en.wikipedia.org/wiki/Pulse-width_modulation) signals accordingly. Programmed with [Arduino](https://www.arduino.cc/) and the [ESP8266 Core](https://github.com/esp8266/Arduino). If you have questions or improvements create a new issue [here](https://github.com/Reinbert/ros_diffdrive_robot/issues).

![front view](https://imgur.com/VKUpHHX.jpg)

## Parts needed:
- Smart car chassis with at least 1 motor on each side, like this: [Smart car chassis 4WD](https://www.aliexpress.com/item/Smart-car-chassis-4WD-4-wheel-drive-force-the-chronological-magnetic-motor-With-code-disc-tachometer/32622219972.html?spm=a2g0s.9042311.0.0.27424c4djmBIqw)
- [NodeMCU](https://en.wikipedia.org/wiki/NodeMCU) (I'm using version 1.0, but v3 is out already)
- L298N motor driver like this: [L298N](https://www.aliexpress.com/item/L298N-Module-Dual-H-Bridge-Stepper-Motor-Driver-Board-Modules-for-Arduino-Smart-Car-FZ0407-Free/1761850243.html)
- Battery pack or other energy source. (I use 8x 1.2 Volt rechargable batteries)
- Breadboard jumper wires
- Linux workstation to run ROS
- Wifi access point (or use the access point from the NodeMcu)
- Gamepad (optional)

## Software:
- [Arduino IDE](https://www.arduino.cc/en/Main/Software)
- [Robot Operating System](http://www.ros.org/)

## Connections:
- Battery (+) to L298N +12V or VCC 
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

## ROS Installation:
- Install and configure ROS if you haven't done so yet ([Installation Tutorial](https://wiki.ros.org/ROS/Installation)). I'm using Kinetic Kame on Ubuntu 16.04, but feel free to change the following terminal commands to your version.
- Install [rosserial_server](https://wiki.ros.org/rosserial_server) via terminal: `sudo apt install ros-kinetic-rosserial-server`
- Install [teleop_twist_keyboard](https://wiki.ros.org/teleop_twist_keyboard) `sudo apt install ros-kinetic-teleop-twist-keyboard`
- Or if you have a gamepad install [teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy): `sudo apt install ros-kinetic-joy ros-kinetic-teleop-twist-joy`

## Arduino IDE:
- Go to _"Sketch/Include Library/Manage Libraries..."_, search for _"ESP8266"_ and install that library.
- Use the same method to install library _"rosserial"_ or if that doesn't work follow this [tutorial](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).
- If you want to use your own Wifi remove the line containing `#define ACCESS_POINT_SSID "SMARTCAR"` and add your own Wifi credentials into `setupWiFi()`
- Find out the IP address of the computer you intend to use as the ROS serial server and change `IPAddress server(192, 168, 4, 2);` accordingly. (You may need to connect to the `SMARTCAR` Wifi network first if you are using access point mode.)
- If you are using a different micro controller than the NodeMcu v1.0 find out what its highest possible PWM value is and change 'MAX_PWM' and 'MIN_PWN' accordingly. (Max PWM on Arduino is 255, while on a NodeMcu it's 1023)
- Flash _nodemcu-ros-car.ino_ onto the NodeMcu

## ROS Operation:
- Launch rosserial socket node: `roslaunch rosserial_server socket.launch`
- Launch gamepad node in a new terminal: `roslaunch teleop_twist_joy teleop.launch joy_config:=__insert gamepad type__` or 
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
Launch teleop AND a rosserial socket at the same time. With this you don't need to launch a [rosserial_server](https://wiki.ros.org/rosserial_server) in a second terminal:
```sh
roslaunch teleop_joy_race teleop_serial.launch 
```
Disable deadman button. Only intended for toy cars. USE AT YOUR OWN RISK!
```sh
roslaunch teleop_joy_race teleop.launch enable_deadman:=false 
roslaunch teleop_joy_race teleop_serial.launch enable_deadman:=false
```
## Troubleshooting:
- Open Serial Monitor in Arduino IDE and check for Wifi and ROS connection output. 
- The LED on the NodeMcu should turn on (only) if its connected to both Wifi and ROS.
- [teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy) and [teleop_joy_race](https://github.com/Reinbert/teleop_joy_race) use an enable (deadman) button which needs to be pressed while using the gamepad. Find out which one in the [config](https://github.com/ros-teleop/teleop_twist_joy/tree/indigo-devel/config).
- Test gamepad with `jstest /dev/input/js0`
- Make sure the teleop node is outputting commands: `rostopic echo /cmd_vel`
- If the LED on the L298N isn't lit, then the batteries aren't connected properly or the jumper for the 5V regulator isn't placed.


## Images:
The cables and battery packs are a little bit whacky, but it's sufficient for the proof of concept.
![full view](https://i.imgur.com/knOrHnc.jpg)
![front view](https://imgur.com/VKUpHHX.jpg)
![detail connections NodeMcu and L298N](https://i.imgur.com/jP4KJ6y.jpg)
![detail cables to motors](https://i.imgur.com/0ckEcW2.jpg)
#### NodeMcu GPIO pins
![NodeMcu pinout](https://pradeepsinghblog.files.wordpress.com/2016/04/nodemcu_pins.png?w=616)
#### Example L298N motor driver
![L298N description](http://qqtrading.com.my/image/catalog/Products/Module/L298N-module/L298N_Motor_Driver_Connections.jpg)
