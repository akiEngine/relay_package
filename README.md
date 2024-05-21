# ----- ROS2 package to control serial communication multi chanels relays----

This package is based on a multi-chanel relay (for this case 2 chanels). The model of the relay is a USB-RELAY02 from SEEIT (doc available on their website). the principle of the relay is to receive bytes and depending on the byte, change the state of one or more relays. 
For example, this relay is a 2 chanels, so there is 4 different bytes to send to get the 4 states (off off, off on, on off, on on). 

## 1 - install

- Install ROS2
- Install this package with:
```
git clone https://github.com/akiEngine/relay_package.git
source install/setup.bash
```
## 2 - run

To launch the node, you can run 
```
ros2 run relay_control server
```

## 3 - use

I made this package in a way you can use it with an action client or with publishing on a topic. If you constantly know the state of the relay you don't want to control, you can use the action server and send a goal with the state of both relays. 

If you don't know, you can publish on either topics "relay1" or "relay2" and it will change only the state of the relay you want to change.

### 1 - action_server mode
to send a goal to the action_server, you can either code an action client or use
```
ros2 action send_goal /relay_control relay_control_interfaces/action/Relay "{r1: false, r2: false}"
```
### 2 - topic mode
to change only one of the relays, you can use a classic publisher or run
```
ros2 topic pub /relay2 std_msgs/msg/Bool data:\ false\ 
```