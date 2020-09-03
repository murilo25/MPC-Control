# MPC-Control

This project implements a Model Predictive Controller in C++. It uses Udacity Term 2 simulator to generate vehicle telemetry data (x and y position, heading angle, velocity as well as throttle and steering angle commands).

The project is composed by the following:

main.cpp:
* Interfaces the project with Udacity Term 2 simulator.
* Calculates cross-tracking error and heading error given vehicle's telemetry data. 
* The simulator also provides a set of x,y coordinates in map frame that describe the vehicle's intended trajectory. These coordinates are converted to vehicle frame and a 3rd order polynomial is used to interpolate between these coordinates.
* To include actuator delays, the states of the vehicle at time `t + actuator_delay` are calculated and passed to the MPC controller.
* After `mpc.Solve()` returns a sequence of the optimized states of the vehicle, the control actuation for the next time instant is fed to the simulator along with the x and y sequences for visualization purposes.

MPC.cpp: 
* Defines a MPC class that is responsible for defining prediction horizon, cost function (`fg[0]`) and performing an optimization constrained to the vehicle kinematics, error dynamics and actuator limits.

* Defines a cost function that has as objective, minimize cross-tracking error, heading error, use of actuators, acceleration and maintain reference velocity.
