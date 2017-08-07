# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo. (see [original udacity repo here](https://github.com/udacity/CarND-MPC-Project))
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

---

## The Model
#### Describe model in detail. This includes the state, actuators and update equations.

The MPC consists of:
1. Vehicle trajectory with number of timesteps `N` and timestep duration `dt`.
2. Vehicle state and actuation variables, along with lower/upper constraints on the variables. State and actuation consists of: 
- `x,y` positions
- `psi` yaw angle
- `v` velocity
- `cte` cross track error
- `epsi` yaw error
- `delta` steering angle
- `a` acceleration
3. Cost function to optimize actuation for 2 objectives: __(1) Speed__ close to desired speed, __(2) Trajectory__ close to polynomial line of reference path. The cost function utilizes state variables as well as the value gap between sequential actuator cost (`deltadot`, `adot`).

The cost function weights are taken from the [project Q&A video](https://www.youtube.com/watch?v=bOQuhpz3YfU) after some experimentation with other weighting factors that heavily penalized `delta` & `deltadot` rather than the current heavy weighting on `cte` & `epsi`, as seen below:
```
cte_wt      = 2000;  // cross-track error
epsi_wt     = 2000;  // psi error
v_wt        = 1;     // reference velocity
delta_wt    = 5;     // steering delta
a_wt        = 5;     // acceleration
deltadot_wt = 200;   // steering delta change
adot_wt     = 10;    // acceleration change
```

The state is updated with the following equations:
```
x = x + v * cos(psi) * dt
y = y + v * sin(psi) * dt
psi = psi + (v/Lf) * delta * dt
v = v + a * dt
cte = cte + v * sin(epsi) * dt
epsi = epsi + (v/Lf) * delta * dt
```


## Timestep Length and Elapsed Duration (N & dt)
#### Discuss the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Provide details of previous values tried.

The values were initially set to `N=10` and `dt=0.1`, but the timestep length `N` value was later increased to 16 to account for additional points projected into the future. The elapsed duration `dt` value was increased to 0.11 to be larger than the 0.1s latency period.

After these values failed to impact the controller's results, the values were returned to the original `N=10` and `dt=0.1` settings from the Q&A video.


## Polynomial Fitting and MPC Preprocessing
#### A polynomial is fitted to waypoints. If preprocess waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The yellow reference line in the video indicates a 3rd degree polynomial fitted with waypoints received from telemetry, while the green line indicates the MPC predicted trajectory.

Prior to the MPC updates, the waypoints are in global coordinates and need to be shifted to adopt the vehicle's frame of reference. The vehicle's location is transformed and the yaw angle rotated in `main.cpp` lines 104-110). 


## Model Predictive Control with Latency
#### Implement Model Predictive Control that handles a 100 millisecond latency. Provides details on how to deal with latency.

The MPC handles a 100 millisecond latency (i.e., the car reacts to actuation after 100ms) by using a predicted state that is calculated 100ms into the future. 
- This "future" state uses the same update equations in the MPC and adjusts the `x, y, psi, v` variables in `main.cpp` lines 128-135. 
- The new state is passed to `mpc.Solve` and returns actuation variables that will properly match a state 100ms in the future.


Using the MPC the vehicle successfully drives around the track, and two completed laps can be seen here:

[![MPC](http://img.youtube.com/vi/nBQMfh9YI1k/0.jpg)](https://youtu.be/nBQMfh9YI1k "MPC")
