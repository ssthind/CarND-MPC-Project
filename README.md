# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
This git repository contains the implementation of Model-Predictive-control to control the actuators steering and throttle/brake of the car for simulator term-2 project 5.

## Rubric Points discussions

### The Model
The includes the state, actuators and update equations
#### State
Consists of the following 6 dimensions
x   : x coordinate in car-coordinate system
y   : y coordinate in car-coordinate system
psi : heading direction of car
v   : velocity in heading direction
cte : deviation of center of car for center of road 
epsi: deviation in required and current heading direction

#### Actuators
Steering angle: `-delta` . this should be in range [-25, 25] degrees. Positive Steering indicates right side and negative Steering indicates left side steering. 
Throttle: `a`. this should be in range [-1, 1]. Positive throttle indicates accleration and negative throttle indicates breaking.

#### update equations
'''
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * (-1) * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] *(-1) * delta[t] / Lf * dt
'''
here Lf : distance between center of car and front axle

### Time lenght and lapsed duration (N & dt)
N : future sample considered during solving of MPC optimation problem. Final value of 10 choosen, other higher values resulted in processing delay(degrading computational performace and unknown processing latency) and lower values does NOT optimatize better of path/curve.
dt : sampling interval duration, setting equal to latency to simplefy considering latency between actuator application and response

### Polynomial fitting and MPC preprocessing
Polynomial of degree/order 3 was used to fit the waypoints provided by simulator.
Waypoints with pre-processed also to change from global map coordinates to car coordinates. 

### Model Predictive control with latency
The actuator controls are applied to the model update equation after delay of 100ms(as dt=0.1s) to the state after the current states in code line 107 to 127 defined in `operation function` of `class FG_eval` in file MPC.cpp
`operation function` also defines the Cost function as `fg[0]` which we mininized for the optimation solution produced by function `CppAD::ipopt::solve()` in `MPC::Solve()`.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

