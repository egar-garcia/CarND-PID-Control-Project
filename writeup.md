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

The Twiddle optimization algorithm presented in the lessons was also integrated in the solution (implemented in ```PID.cpp``` lines 74 to 122) to do continuous tuning of the given parameters, samples of a given length are considered (samples of size 100 was chosen). The delta to modify the parameters is reduced and converging to zero as long as the method is performed, reaching a point where the parameters are not longer changed (or is not noticeable).


### Reflection

#### Describe the effect each of the P, I, D components had in your implementation.
##### Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

###### Coefficient P (position)

Coefficient P is used to steer in opposite direction of the CTE, in order to reduce the CTE the next time lapse, however by itself it can make the car to continuously oscillates, as can be seen in this video (with a value of 30): [P coefficient  example](./docs/PID_controller_P.mov).

![video_P]

###### Coefficient D (differential)

Coefficient D is useful to avoid oscillations, by counter steering when the CTE is being reduced or add up to the steering direction if the CTE is being increased. In this video it can be seen how used by itself (with a value of 0.35) mitigates the oscillations: [D coefficient  example](./docs/PID_controller_D.mov).

![video_D]

###### Coefficient I (integral)

Coefficient I (integral) is useful to correct a possible bias involved in the measurement of the CTE in long term runnings.

In this video the coefficient I is set to zero: [Without I coefficient  example](./docs/PID_controller_no_I.mov)., as it can be seen the CTE has a tendency to stay slightly on the positive side around 0.07.

![video_no_I]

In this video the coefficient I is set to 0.01: [I coefficient  example](./docs/PID_controller_I.mov)., as it can be seen the CTE has bounces around zero, indicating that the bias has been reduced.

![video_I]

###### PID for throttling

To get the value of the throttle a PID controller was also used, for this case an intentional bias is selected of 0.5, the intuition behind is that it would be a safe to drive at half throttle (since it gives more flexibility to stress it up or down).

In this case, similarly, the coefficients P and D are applied to the magnitude (absolute value, since the direction doesn't matter), the idea is that the more the error the less the throttle (even it might be cases where is necessary to break), and when the error is low the throttle would be higher.


#### Describe how the final hyperparameters were chosen.
##### Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

Initially I chose the steering values ```P = 0.2```, ```D = 5.0``` and  ```I = 0.001```. As describe before, my implementation also involves doing continuous Twiddle, so, I performed several manual tests, specially for tuning the coefficient D, combined with Twiddle. At the end I chose the parameters that presented the best results during my tests ```P = 0.35```, ```D = 30.0``` and ```I = 0.01```. I left some parameter's deltas of 10% of the parameters' values to still doing twiddle while the procedure is running.

The election of the parameters for the throttle's PID was a bit more difficult, and required more manual testing and intuition. First, I experimented with a direct relation of throttle 1.0 if error was 0.0 and vice versa, but the car lost control due to the speeds it reached and the sudden approach to a curve. So at the end the intuition was that in general it would be safer and relaxing to drive at half throttle, and from there to adjust according to the CTE. The final values selected where ```P = 0.3```, ```D = 0.3``` and ```I = 0.0``` (I is zero since the absolute value of CTE is considered and a bias was introduced on purpose) with a bias of 0.5 (half throttle), again with deltas of 10% to allow some Twiddle during the execution.

### Simulation

#### The vehicle must successfully drive a lap around the track.
##### No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

After the previously described implementation of the PID controller, the execution in my computer using the Term 2 Simulator was able to successfully drive the car within the track surface.

This can be seen in the following video of two laps: [run example](./docs/PID_controller_run.mov), the second lap runs smother that the first one thanks to the effect of the integrated Twiddle in action.

![video_run]
