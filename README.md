## Udacity Term 2 Project 1 : Extended Kalman Filter

[//]: # (Image References)

[image1]: CARND-Term2-Project1/img/Capture_FusionLidar&radarDataSet1.PNG "Test1"
[image2]: ./img/Capture_FusionLidar&radarDataSet2.png "Test2"
[image3]: ./img/Capture_OnlyRadarData1.png "Test3"
[image4]: ./img/Capture_OnlyRadarData2.png "Test4"

![alt tag](CARND-Term2-Project1/img/Capture_FusionLidar&radarDataSet1.PNG)

The goal of this project is to use a [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) to estimate the state of a moving object of interest with noisy lidar and radar measurements.

The program, is written in C++, has the following steps:
1. Takes noisy Lidar and Radar measurements about the position of the moving object using WebSocketIO from simulator.
2. State and covariance matrcices are initialized at first initial measurement.
3. New measurements are used to predict the new state and then measurement update steps is done for Lidar and Radar to get new estomate,
   in the ProcessMeasurement function of FusionEKF class.
4. Calculates RMSE comparing the estimations with the ground truth values--Passing the project requires obtaining RMSE values below 0.11 for `x` and `y`, and below 0.52 for `v_x` and `v_y`.

## Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build Instructions

First, clone this repo.

This project can be used with a Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

In my project I'm using Docker on windows, so followed the below steps:

Docker on Windows
The best place to start is to follow the instructions here. A common pitfall is to not log in to the docker container. The first time you run docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest the controls_kit may download, but the system may not log you in to the container. If you are logged in instead of a $ prompt, you should see something like this: root@27b126542a51:/work#. If you are not logged into the container commands such as apt-get and make will not be recognized, so be sure to execute docker run -it -p 4567:4567 -v 'pwd':/work udacity/controls_kit:latest again, if you do not see the correct prompt.


1. Make the build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./ExtendedKF
4. Run the Simulator to execute Data Set 1 & Data Set 2

## Program Files changed in the source code:
Contained in `src` folder:
* `main.cpp`: reads in data, stores the data into a measurement object, calls a function to run the KF, calls a function to calculate RMSE.
* `FusionEKF.cpp`: initialize variables and matrices (x, F, H_laser, H_j, P,R_laser_,R_radar_ etc), initializes KF position vector, calls the predict function, calls the update function (for either the lider or radar sensor measurements)
* `kalman_filter.cpp`: defines the predict function, the update function for lidar,update function for radar & function to claculate polar cordinates from cartesian.
* `tools.cpp`: calculate RMSE, the Jacobian matrix and the Polar-to-Cartesian transformation functions.

How the files interact when you run the C++ program:
* `main.cpp` reads in sensor data (line by line) from the client (simulator) and sends a sensor measurement to FusionEKF.cpp
* `kalman_filter.cpp` and `kalman_filter.h` define the *KalmanFilter* class.
* `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. It has a variable `ekf_` that's an instance of a KalmanFilter class. `ekf_` will hold the matrix and vector values. We also use the `ekf_ `instance to call the predict and update equations.

## Results:
Below is the output of my EKF from two different simulated runs using the input data provided.

**Test 1**: input Simulator Data Set 1, Both Lidar and Radar Measurements

| Input |   RMSE  |
|:-----:|:-------:|
|  px   | 0.0973  |
|  py   | 0.0855  |
|  vx   | 0.4513  |
|  vy   | 0.4399  |

[image1]

**Test 2**: input  Simulator Data Set 2, Both Lidar and Radar Measurements

| Input |   RMSE  |
| ----- | ------- |
|  px   | 0.0726  |
|  py   | 0.0967  |
|  vx   | 0.4579  |
|  vy   | 0.4966  |

![alt text][image2]

## Some additional test done with only radar measurement

**Test 3**: input  Simulator Data Set 1, **Only Radar Measurements**

| Input |   RMSE  |
| ----- | ------- |
|  px   | 0.1918  |
|  py   | 0.2798  |
|  vx   | 0.5575  |
|  vy   | 0.6567  |

![alt text][image3]


**Test 4**: input  Simulator Data Set 2, **Only Radar Measurements**

| Input |   RMSE  |
| ----- | ------- |
|  px   | 0.2244  |
|  py   | 0.2954  |
|  vx   | 0.5870  |
|  vy   | 0.7338  |

![alt text][image4]
