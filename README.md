# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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

## Model Used
The model described in the lectures was used, and the codes were heavily based on the track the line quiz from the lectures

The car drives on a track, and the path is approximated by a third order polynomia.
The coordinates provided are in the map frame, while for this particular project, I have converted the coordinates in the car frame of reference, speed and orientation are also given
The benefit of doing so is, that in the car's frame, the initial coordinates are (0,0,0) for (x,y,psi)
The waypoints recieved from the car, are converted into the car frame, and then a third order polynomial is fit, to handle the cross track error, the steering angle and the speed of the car

My optimizer takes the following aspects in consideration:

I) Based on the reference state
 1) Cross track error, with a factor of 1500 (Very heavy penalization for increasing CTE)
 2) Orientation of the car, with a multplicative factor of 1500 (Very heavy penalization)
 3) Difference of the car velocity and the reference velocity
 
II) Minimize the use of actuators --> This is so that we do minimum work and have a smooth ride
 1) Change in steering angle 
 2) Acceleration

III) Smooth response of the actuators --> We do not want the actuator values to fluctuate very heavily
 1) Steering angle change between two timestamps
 2) acceleration difference between two timestamps
 
 ## Timestamp and Frequency
 The length of the timestamp is chosen as 10 and the frequency is 0.15 (although the model works fine for 0.10 as well)
 
 ## Latency 
 Model can handle latency of upto 100 milliseconds
 
