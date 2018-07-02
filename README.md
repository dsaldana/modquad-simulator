# ModQuad and Crazyflie Simulator

### Requirements
* Install crazyflie_ros: https://github.com/whoenig/crazyflie_ros
* install python packages `$ sudo pip install --upgrade numpy scipy transforms3d`



### Launching the simulator
The simulator for five quadrotors is launched by
```
$ roslaunch modquad_simulator simulation.launch
```
Each robot simulates the dynamics of a quadrotor by runing the `modquad_sim` node and its pose is displayed in RViz.
The launch file `simulation.launch` can be easily modified to change the number of quadrotors, their initial location and their color.

As well as the actual crazyflie robot, the simulator receives two inputs:
* Attitude command: the topic _cmd_vel_ receives the desired thrust, roll, pitch and yaw inputs. It follows the same format as the `crazyflie_ros `package.
* Goal: using the `crazyflie_controller`, the simulator also receives goals trhough the _goal_ topic and takeoff commands through the _takeoff_ service. 

### Demo
Once the simulator is running, we can send desired goals to the robots.  The following command runs a demo script that 
takes off the robots and makes them move in circle.
```
rosrun demo-simulator demo_circle_multiple.py
```










### Credits
This is an open source project, most of the code is licensed under GPL v3.0.

This simulator and is developed and maintained by [David Saldaña](http://davidsaldana.co/) at University of Pennsylvania.

Part of the dynamics simulation is an adaptation of the Quadrotor Matlab Simulator by Daniel Mellinger at University of Pennsylvania.
 

