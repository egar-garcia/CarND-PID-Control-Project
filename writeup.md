**PID Controller Project**


Self-Driving Car Engineer Nanodegree Program

In this project the lake race track from the Behavioral Cloning Project is revisited. This time, however, implementing a PID controller in C++ to maneuver the vehicle around the track!

[//]: # (Image References)
[video_P]: ./docs/PID_controller_P.mov
[video_D]: ./docs/PID_controller_D.mov
[video_no_I]: ./docs/PID_controller_no_I.mov
[video_I]: ./docs/PID_controller_I.mov
[video_run]: ./docs/PID_controller_run.mov



### Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./pid
```


## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### Your code should compile.
##### Code must compile without errors with ```cmake``` and ```make```. Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

As the file ```CMakeLists.txt``` was made as general as possible (including necessary steps for compiling in Mac, which is the platform I used to work in this project) I left if unchanged. Thus, after setting up [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), the compilation and building can be done using the standard procedure (from the project's top directory):

```
mkdir build
cd build
cmake ..
make
./pid
```


### Implementation

#### The PID procedure follows what was taught in the lessons.
##### It's encouraged to be creative, particularly around hyperparameter tuning/optimization. However, the base algorithm should follow what's presented in the lessons.

The used PID procedure follows the one taught in the lessons based in the following formula (implemented in ```PID.cpp``` lines 46 to 64):

```
pid_control_output = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
```

Where:
* ```CTE``` is the cross track error.
* ```diff_CTE``` is the differential cross track error given by ```CTE(t) - CTE(t-1)```.
* ```int_CTE``` is the integrated cross track error given by ```sum(CTE(t))``` for all the previous CTEs.
* ```tau_p```, ```tau_d``` and ```tau_i``` are the parameters used to define the influence of the previous ones.

The Twiddle optimization algorithm presented in the lessons was also integrated in the solution (implemented in ```PID.cpp``` lines 74 to 122) to do continuous tuning of the given parameters, considering samples of a given length (samples of size 100 was chosen). The delta to modify the parameters is reduced and converging to zero as long as the method is performed, reaching a point where the parameters are not longer changed (or is not noticeable).


### Reflection

#### Describe the effect each of the P, I, D components had in your implementation.
##### Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

Coefficient P (position) is used to steer in opposite direction of the CTE, in order to reduce the CTE, however by itself it can make the car to continuously oscillate as seen in this video: [P coefficient  example](./docs/PID_controller_P.mov).

![video_P]

Coefficient D (differential) is useful to avoid oscillations, by counter steering when the CTE is being reduced or add up to the steering direction if the CTE is being increased.

![video_D]

![video_no_I]

![video_I]


#### Describe how the final hyperparameters were chosen.
##### Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!



### Simulation

#### The vehicle must successfully drive a lap around the track.
##### No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).
