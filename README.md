# Model-Predictive-Control
An implementation of the Model Predictive Control to drive the car around the track.

## Description ##
The Model Predictive Control defines a task of keeping a vehicle on its trajectory as an optimization problem. This problem resumes to a simulation of different actuators' inputs, a prediction of results in trajectory and selecting the option with a minimal cost.

#### The Model ####
The model that combines the previous state and the actuators from previous timestep were used to obtain the current state as following:
<br>
![x_t](http://www.sciweavers.org/tex2img.php?eq=x_%7Bt%7D%20%3D%20x_%7Bt-1%7D%20%2B%20v_%7Bt-1%7D%20%2A%20cos%28%5Cpsi_%7Bt-1%7D%29%20%2A%20dt&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)<br>
![y_1](http://www.sciweavers.org/tex2img.php?eq=y_%7Bt%7D%20%3D%20y_%7Bt-1%7D%20%2B%20v_%7Bt-1%7D%20%2A%20sin%28%5Cpsi_%7Bt-1%7D%29%20%2A%20dt&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)<br>
![psi_[t]](http://www.sciweavers.org/tex2img.php?eq=%5Cpsi_%7Bt%7D%20%3D%20%5Cpsi_%7Bt-1%7D%20%2B%20%5Cfrac%7Bv_%7Bt-1%7D%7D%7BL_%7Bf%7D%7D%20%2A%20%5Cdelta_%7Bt-1%7D%20%2A%20dt&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0) <br>
![v_[t]](http://www.sciweavers.org/tex2img.php?eq=v_%7Bt%7D%20%3D%20v_%7Bt-1%7D%20%2B%20a_%7Bt-1%7D%20%2A%20dt&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0) <br>
![cte[t]](http://www.sciweavers.org/tex2img.php?eq=cte_%7Bt%7D%20%3D%20f%28x_%7Bt-1%7D%29%20-%20y_%7Bt-1%7D%20%2B%20v_%7Bt-1%7D%20%2A%20sin%28E%5Cpsi_%7Bt-1%7D%29%20%2A%20dt%20&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)<br>
![epsi[t]](http://www.sciweavers.org/tex2img.php?eq=E%5Cpsi_%7Bt%7D%20%3D%20%5Cpsi_%7Bt%7D%20-%20%5Cpsi%20des_%7Bt-1%7D%20%2B%20v_%7Bt-1%7D%20%2A%20%5Cfrac%7B%5Cdelta_%7Bt-1%7D%7D%7BLf%7D%20%2A%20dt&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)<br>
, where x and y are vehicle's coordinates, psi is an orientation angle, v is a velocity, cte is a  cross-track error and epsi is a psi error. Actuator outputs are described by a - an acceleration and a steering angle - delta.

The cost error between a prediction and a measurement was minimized by solving third degree polynomial with Integer Linear Programming solver from Ipopt/CppAD packages.

#### Timestep Length and Elapsed Duration ####
Following Udacity's suggestion and trial-and-error appetites, 15 was chosen for the `N` and 0.1 for the `dt`.

#### Polynomial Fitting and MPC Preprocessing ####
Again, following Udacity's suggestion, the waypoints were transformed to vehicle's perspective. It simplifies the process to fit a third degree polynomial considerably.

#### Model Predictive Control with Latency ####
The latency handling was implemented in `main.cpp` file; details can be found here, from [line 127](https://github.com/antonpavlov/Model-Predictive-Control/blob/344ee915e0d5b8a750e7b645e295af66c156accf/src/main.cpp#L127) through [line 138](https://github.com/antonpavlov/Model-Predictive-Control/blob/344ee915e0d5b8a750e7b645e295af66c156accf/src/main.cpp#L138).

## Requirements ##
In order to successfully build and run the program, the following requirements should be fulfilled:
* `cmake` equal or above version 3.5
* `make` equal or above version 4.1
* `gcc/g++` equal or above version 5.4
* [`uWebSocketIO`](https://github.com/uNetworking/uWebSockets) library
* `Ipopt` and `CppAD`. These two math packages may become a pain to install if the [document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) is not followed thoroughly.

[Initial Udacity's repo](https://github.com/udacity/CarND-MPC-Project) is a good starting point for setting up all necessary packages.

## Building and Running the code ##
In order to compile, please execute the following commands in a project folder:
* `cmake ./` - configuration of build; in case of any significant changes, you may want to delete `CMakeChache.txt`
* `make` - actual build
* `./mpc` - an execution of the program

Program opens the port 4567 (configurable in `src/main.cpp`) and waits for a connection of the simulator described in the Validation section below.

## Validation ##
Once compiled and launched, the program tries to establish a connection with a simulator. The **Term 2 Simulator** software includes a graphical simulator with a track and a car. The simulator can be downloaded here: [https://github.com/udacity/self-driving-car-sim/releases](https://github.com/udacity/self-driving-car-sim/releases)

The result of this implementation of the Model Predictive Control driving a car around the track can be seen on the following link: [YouTube](https://youtu.be/W0A9jG9cjn4)

The data exchanged between simulator and the program is described here: [Websocket Data](https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md).

The contents of this repo were tested in Ubuntu Linux 16.04.


## License ##
All software included in this repository is licensed under MIT license terms. All additional programs and libraries have their owners and are distributed under their respective licenses. This repository contains Udacity's intellectual property. For any inquires on its reuse or commercialization, please contact Udacity at [www.udacity.com](https://www.udacity.com/).
