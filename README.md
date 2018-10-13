# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Overview

The project contains the implementation of [PID controller](https://en.wikipedia.org/wiki/PID_controller) for Udacity Self Driving Nanodegree program. The target of the PID controller was to adjust primarily the steering angle of an autonomous car simulated in the [Udacity SD Simulator](https://github.com/udacity/self-driving-car-sim/releases). Furthermore the controller was used to additionaly adjust the throttle of the simulated car.

## Reflection

### Selection method

The most intensive task of the project was the tuning of the gains for the PID controller. For this task two different approaches were combined.

* First a manual method was followed in which:
    1. Start with all gains set to 0
    2. Increase `Kp` until the system oscillates
    3. Increase `Kd` until the oscillation is critically damped
    4. Repeat 2-3 steps until the oscillation cannot be damped by the increased `Kd`
    5. Increase `Ki` keeping in mind the tradeoff between fast response time (quick turns or acceleration in our occasion) and the oscillations

* After defining manually a good set of parameters the fine tuning was done by using Twiddle algorithm which was taught in the classroom. (PID.cpp line 124-175). 

At first Twiddle algorithm was used from the beggining (initial gains set to 0). But the algorithm tends to find the local minima which were not fit for the plant. The other problem is that without a good set of initial parameters the car tends to leave the track and this invalidates the optimization.

Firstly the PID controller of the steering angle was calibrated with a constant throttle of 0.3. After the parameters were finalized the PID controller of the throttle was also calibrated seperately.

### Observations

The proportional term is directly connected to the actual CTE (Cross Track Error or the distance from the lane center ). Therefore this term has the most observable effect on the steering behavior. But using only this term the car oscillates between the lane center constantly.

By adding the differential term the car stops oscillating so harsh along the lane center. This is normal as the D-term counteracts the P-term. However, even if the oscillation is reduce the car is not approaching the lane center but an offset/bias of it.

Adding the integral term fixes the offset/bias problem and this is more obvious during turning at the curves.

In the [Final](https://github.com/niktolis/SDCar-P9-PIDControlProject/blob/master/video/Final.mp4) video both the controllers are working for a complete track run.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

