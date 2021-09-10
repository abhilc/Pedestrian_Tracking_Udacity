# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found in the classroom lesson for this project.

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Extended Kalman Filter - Algorithm

Goal: With the incoming measurements from two sensors(Radar and Laser), Implement a filter such that the vehicle tracked is as close as possible to the ground truth data. To measure the error of Kalman Filter Prediction and Update Step, Root Mean Square Error(RMSE) is computed. The allowed benchmark for RMSE for position(px, py) and velocity vector(vx, xy) is RMSE <= [.11, .11, 0.52, 0.52]

Following steps were followed in the implementation of the algorithm:
1. In file FusionEKF.cpp, the following Kalman Filter Matrices are initialized in the class' constructor method.
   
   R_laser : Measurement Noise Matrix for Laser data
   R_radar : Measurement Noise Matrix for Radar data
   H_laser : Matrix that converts the predictions to laser measurement space(Drops the velocities from the prediction state vector)
   H_j     : Jacobian Matrix. This matrix contains partial derivatives(Linear approximations) to convert prediction state vector to Radar measurement space
   P Matrix: This Matrix is a state covariance matrix. 

2. In file main.cpp, the measurements from Laser and Radar are copied to the measurement_package structure. The method ProcessMeasurements is called(This is defined    in FusionEKF.cpp. 

3. In method ProcessMeasurements:
   a. When the very first measurement arrives, The state vector X is initialized. The timestamp of the first measurement is recorded. 
   b. From second measurment onwards, the time difference dt is computed. The state transition matrix F, the covariance Matrix Q are initialized
   c. Depending on the type of this measurement(Laser or Radar), The measurement noise matrices(R_laser or R_radar), The measurement update matrices(H_laser or           H_jacobian) are assigned to Kalman Filter variables. 
   d. The Predict() method is called to Predict the State Vector
   e. The Update() method is called to Correct the State Vector is the incoming measurement is Laser, otherwise UpdateEKF() method is called. 

4. The methods Predict(), Update() and UpdateEKF methods are defined in kalman_filter.cpp file

5. The Predict() method implements the Kalman Filter Prediction Equations(See kalman_filter.cpp Predict() method for implementation)

6. The Update() method implements the correction step for Laser. Here, the predicted state vector is corrected by computing the Kalman Gain, K. 

7. The UpdateEKF() method implements the correction step for Radar. Here the predicted state vector is corrected by computing the Kalman Gain, K. 

8. In tools.cpp file, Functions that compute RMSE(Root Mean Squared Error) and Jacobian Matrix(H_j) are implemented. 

9. After each Prediction and Update step, the RMSE method is called with arguments(estimation and ground_truth). This method then returns by how much the              estimations deviate from the ground truth. 

## Simulator Results

For DataSet 1, it can be seen from the results that RMSE are within the allowed benchmark

<p>
  <img src="writeup_images/Screenshot 2021-09-10 at 12.13.39 PM.png"/>
</p>


For DataSet 2, it can be seen from the results that RMSE are within the allowed benchmark
<p>
  <img src="writeup_images/Screenshot 2021-09-10 at 12.14.16 PM.png"/>
</p>

