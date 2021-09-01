# agv_base_control
This repository is used to develop a ROS base controller with STM32 for AGV. Noted that this repository is developed and tested by ROS-Noetic and Ubuntu Server 18.04 64 bit in raspberry pi 3B+.

## Installation
Please install the following packages when ROS Noetic is installed properly.
1. ROS teleop_twist_keyboard
```
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

2. ROS rosserial
```
sudo apt install ros-noetic-rosserial
```

3. ROS rosserial_stm32
```
cd src
git clone https://github.com/yoneken/rosserial_stm32.git
cd ..
catkin_make
```

## Compile STM32CUBMX code
1. User chatter as an example
```
cd agv_base_control/src/rosserial_stm32/src/ros_lib/examples
sudo cp -avr chatter/ ~/development/STM32_cubmx_program/
cd ~/development/STM32_cubmx_program/chatter
rosrun rosserial_stm32 make_libraries.py .
```
**You may see a question about choosing a non-unique executable, please choose the one in src/rosserial. Usually, it is option 2.

**Please check your make_libraries.py and make sure it is suitable for **python3**.

**The pre-build STM32_cubmx_program is packed into this repository for reference.

## Test rosserial with STM32
**Make sure the STM32 usb port is connected to raspberry pi usb port.**

1) ROSCORE
```
roscore
```

2) Set authority to serial port
```
sudo chmod 777 /dev/ttyACM0  ##Note that ttyACM0 may not fit yours
```

3) Bring up ROS serial node
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

4) Checkout ROS message
```
rostopic echo <your_topic>
```

5) Visualize nodes relationshiop
```
rosrun rqt_graph rqt_graph
```

## System startup procedure
**Make sure the STM32 usb port is connected to raspberry pi usb port.**

This part is very similar to "Test rosserial with STM32". However, some steps are removed because they are not necessary when the connection is confirmed.

1. Activate roscore
```
roscore
```

2. Set authority to serial port
```
sudo chmod 777 /dev/ttyACM0
```

3. Bring up ROS serial node
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

4. Start teleop_twist_keyboard control
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## ROS Node Graph
![image](https://github.com/vincent51689453/IC382-ROS-STM32/blob/noetic-pi/git_image/rosgraph.png)
