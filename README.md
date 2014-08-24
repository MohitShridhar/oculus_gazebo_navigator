Oculus-Rift Gazebo Navigator
=======================
<p align="middle">
    <a href="http://www.youtube.com/watch?feature=player_embedded&v=69O5Ya9Zrpk
    " target="_blank"><img src="images/thumbnail.png" 
    alt="IMAGE ALT TEXT HERE" width="718" height="403" border="10"/></a>
</p>

One of the highlights of the [Gazebo 4.0.0](http://gazebosim.org/blog/gazebo4) release is the added support for the trending VR platform: Oculus-Rift. Oculus-Rift Gazebo Navigator is a simple tool that exploits the capabilities of the simulation environment's physics-engine to provide you with the ultimate virtual-reality experience in Gazebo. So simulation will no longer be just a part of development, but instead act as a new medium to demostrate your product to your clients in its work environment (in full-scale).

## Dependencies & Prerequisites
**Core**: [ROS Hydro](http://wiki.ros.org/hydro), [Catkin](http://wiki.ros.org/catkin): see [package.xml](package.xml)

**Custom Requirements:** [Gazebo 4.0.0](https://bitbucket.org/TihomRis/gazebo), [oculussdk](https://github.com/MohitShridhar/oculussdk)

**Third-Party Requirements:** [ps3joy](http://wiki.ros.org/ps3joy) (Ubuntu) or [osx_joystick](https://github.com/walchko/osx_joystick) (OS X)

## Installation & Setup

See the [wiki](https://github.com/MohitShridhar/oculus_gazebo_navigator/wiki/1.-Installation-&-Setup) for more details.

## Controls

### PS3 Controller
<p align="middle">
    ![front](images/front_btns.jpg)
    ![back](images/back_btns.jpg)
</p>

### Keyboard Op
Start controller:
```bash
rosrun oculus_gazebo_navigator oculus_keyboard_controller
```
Instructions:
```
'w' forward, 's' back, 'a' left, 'd' right
'z' rotate leftwards, 'c' rotate rightwards
To run, hold 'shift' first and then press the corresponding buttons
'Q' to quit (i.e. Shift + 'q')
```