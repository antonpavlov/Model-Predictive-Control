# Model-Predictive-Control
An implementation of the Model Predictive Control to drive the car around the track.

## Description ##
The Model Predictive Control defines a task of keeping a vehicle on its trajectory as an optimization problem. This problem resumes to a simulation of different actuators' inputs, a prediction of results in trajectory and selecting the option with a minimal cost.

#### The Model ####
The model that combines the previous state and the actuators from previous timestep were used to obtain the current state as following:
```
x_[t]    = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y_[t]    = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi_[t]  = psi[t-1] - v[t-1] / Lf * delta[t-1] * dt
v_[t]    = v[t-1] + a[t-1] * dt
cte[t]   = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t]  = psi[t] - psides[t-1] - v[t-1] * delta[t-1] / Lf * dt
```
, where x and y are vehicle's coordinates, psi is an orientation angle, v is a velocity, cte is a  cross-track error and epsi is a psi error. Actuator outputs are described by a - an acceleration and a steering angle - delta.

The cost error between a prediction and a measurement was minimized by solving third degree polynomial with Integer Linear Programming solver from Ipopt/CppAD packages.

#### Timestep Length and Elapsed Duration ####
From the theory point of view, the Model Predictive Control optimizes control inputs `[delta, a][delta, a][delta, a]` until a low cost vector of control inputs is obtained. This vector resumes to the following: `[delta(1)​, a(1)​, delta(2​), a(2​),..., delta(N−1​), a(N−1)​]`. Thus, it is clear that `N` has a direct relation to computational cost of that optimization process.

The `MPC.cpp` is set in the following way:
* N = 15
* dt = 0.1

Both of these values form a 1.5 seconds horizon, that equals to 66 feet (or 20 meters) for a speed of 30 mph (or 48 kmph).

Setting up `N = 10`/`dt = 0.1` would also be a good option to start a search for a best fit. However, larger values of `dt` may lead to the "discretization error" where actuations are less frequent. It makes of an accurate approximation of a continuous reference trajectory, a very difficult task.

#### Polynomial Fitting and MPC Preprocessing ####
Again, following Udacity's suggestion, the waypoints were transformed to vehicle's perspective. It simplifies the process to fit a third degree polynomial considerably.

#### Model Predictive Control with Latency ####
The PID controller obtains the error in a present state, but the actuation will be applied in a future. One of the main reasons of the latency is a lack of predictability of actuator dynamics. This fact may cause an instability of the systems, especially those without an accurate vehicle model.

In this particular case, the Model Predictive Control includes a latency into account by adding it to a state vector, as shown in the implementation below (Please, see the `main.cpp` file, from [line 127](https://github.com/antonpavlov/Model-Predictive-Control/blob/344ee915e0d5b8a750e7b645e295af66c156accf/src/main.cpp#L127) through [line 138](https://github.com/antonpavlov/Model-Predictive-Control/blob/344ee915e0d5b8a750e7b645e295af66c156accf/src/main.cpp#L138)):
```
// Latency 100ms
const double dt = 0.1;
// Previous steering angle and throttle
const double delta = j[1]["steering_angle"];
const double prev_a = mpc.prev_a;
const double predicted_x = v * dt;
const double predicted_y = 0;
const double predicted_psi = - v * delta / Lf * dt;
const double predicted_v = v + prev_a * dt;
const double predicted_cte = cte + v * CppAD::sin(epsi) * dt;
const double predicted_epsi = epsi + predicted_psi;
state << predicted_x, predicted_y, predicted_psi, predicted_v, predicted_cte, predicted_epsi;
```

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
