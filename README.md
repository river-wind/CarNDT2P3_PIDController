# PID Controller for Autonomous Vehicle
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

Overview
---

Once more relying on the Udacity simulator, this project uses the PID controller method to drive the simulator's virtual car around Track 1.  The PID controller object has been fleshed out in PID.cpp, relying on provided centerline data to drive the car around the track.  PID stands for Proportional, Integral, Differential, the three components of providing a smooth and effecting steering model for the car.  The proportional aspect steers the car towards the centerline based on the measured distance between it and the car.  The Integral portion handles offset, either in measurement of mechanical bias.  The Differential portion of the controller smooths out the strength of turning driven by the Proportional term, to reduce the overshooting which that factor alone tends to generate.  

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

## Code Style

I've attempted to stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) in this project..

## Project Instructions and Rubric

This project's goal was to build a PID controller in C++, using the barebones PID.cpp file as a starting point.  The method closely follows the PID controller design from Sebastian Thrun's Udacity course on the subject, though that was written in Python.

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Rubric

1. Code must compile without errors with cmake and make.  
   cmake and make were used to compile the project files, without errors or warnings returned.

2. The PID procedure follows what was taught in the lessons.  
   The PID process does follow the lesson material.

3. Describe the effect each of the P, I, D components had in your implementation.  
   PID stands for Proportional, Integral, Differential, the three components of providing a smooth and effecting steering model for the car.  The proportional aspect steers the car towards the centerline based on the measured distance between it and the car.  The Integral portion handles offset, either in measurement of mechanical bias.  The Differential portion of the controller smooths out the strength of turning driven by the Proportional term, to reduce the overshooting which that factor alone tends to generate.  

4. Describe how the final hyperparameters were chosen.  
   The hyperparameters for both the steering and throttle PID controllers were chosen by trial and error, aka by hand.  The twiddle algorithm is a simple enough process which could determine the best values, and I had planned to impliment that, however I did not have the time to do so.  I intend to return to this project at a later date and implement twiddle.

5. The vehicle must successfully drive a lap around the track.  
   See the video below!

## Video

Video of successful run of the PID controller around the track.
https://youtu.be/Exx9Bd5fv0Q

## Conclusion

In order to utilize the PID controllers (in this project, one for throttle and one for steering) centerline data must be available and the vehicle's distance from that error determinable.  Once that is possible, PID controllers are a very simple method for non-nausiating drive control along that center path.  That said, a few additions would likely improve the performance of the PID controller; most obviously averaging out steering to smooth the observed behavior, and looking forward a few time steps to anticipate upcoming curves in the centerline path.
