# MPC (Model Predictive Control)
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./images/1.png "Model"
[image2]: ./images/2.png "Errors"
[image3]: ./images/3.png "Transformation"

## Goal

The goal of the project is to implement Model Predictive Control to drive the car around the track using simple kinematic model.

## Model

The project is based on the simple kinematic model.  The vehicle state in kinematic model include for parameters [x,y,ψ,v]

x - x coordinate of the vehicle

y - y coordinate of the vehicle 

ψ - orientation angle

v - speed

Following equations determine state transition for the simple kinematic model:

![alt text][image1]

Where additional parameters are:

Lf -  measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.

a - acceleration input 

δ - steering angle input

[δ,a] are model actuators or control inputs

So, the goal of the MPC is to optimize actuators of kinematic model to follow desired trajectory.

## Polynomial Fitting and MPC Preprocessing


Assumed we have planned path provided (desired trajectory). For this project we get this information from Unity simulator. 
The next step is to fit polynomial which approximate desired trajectory and send it to the control block.

Also, we are getting trajectory in the global coordinates, however it is beneficial to convert trajectory to the vehicle coordinates (where vehicle coordinates are (0,0) and orientation along X axes - ψ = 0). This simplifies error calculations to the simple equations below:

![alt text][image2]

So, before passing trajectory to the Model we preprocess coordinates to convert them to the vehicle coordinates. Here is good illustration how to achive this:

![alt text][image3]

## Latency 

Additionally, as we have latency between the time when we receive information about the environment and actual control commands are executed - car continue to use old control inputs for this period and changing its position and orientation.  So, we need to apply kinematic model equations to predict actual car state rather than use state which was [latency] seconds earlier to build more accurate model:

```c++
 double px_predicted = v * latency;
 
 double py_predicted = 0;
 
 double psi_predicted = -v * steer_value * latency / Lf;
 
 double v_predicted = v + throttle_value * latency;
```

 Also need to calculate predicted initial errors based on new predicted car state:
 
 ```c++
 double cte = py_predicted - polyeval(coeffs, px_predicted);
  
 double epsi = psi_predicted - atan(coeffs[1] + 2 * coeffs[2] * px_predicted + 3 * coeffs[3] * px_predicted * px_predicted);
```

Additionally, having latency specified in seconds and getting speed from sensors  in mhp we need to convert speed to m/s to correctly predict vehicle state and fit model.

So now our state is ready to be passed to MPC solver:


 ```c++
  Eigen::VectorXd state(6);
  state << px_predicted, py_predicted, psi_predicted, v_predicted, cte, epsi;

  auto vars = mpc.Solve(state, coeffs);
```




## Paramertes Tuning

Also there are couple model parameters to tune making model driving smooth:

```c++
#define N_Points 10
#define dt 0.1

//Define Cost penalties
#define CTE_PENALTY  1
#define EPSI_PENALTY  10
#define SPEED_PENALTY  2

#define STEER_PENALTY  0
#define A_PENALTY  0

#define STEER_CHANGE_PENALTY 15000
#define A_CHANGE_PENALTY  200
```

Firstly, I selected number of points to predict controls input for: N_Points and interval for the points dt.  The final values are N = 10 and dt = 0.1. It means that we are looking ahead 1 for second.  Setting this value higher than one second, making model behavior smoother on the straight lines - however model behaves badly on the changing turns. Also setting N higher and reducing dt to still looking one second ahead increase computation complexity without any visual benefit So, I’ve ended up with the values above.

Additionally, after some experiments with cost parameters tuning it appeared that trying to follow right direction (EPSI) without rabid changes in steering angle (STEER_CHANGE_PENALTY) is more important than trying to be closer to the real trajectory. So, this resulted to the fact that EPSI_PENALTY and STEER_CHANGE_PENALTY got a lot higher value than other error terms. 

However, we still need to use CTE_PENALTY to make smooth corrections to real trajectory. SPEED_PENALTY is used to make car be driving with desired speed.  A_CHANGE_PENALTY is important to prevent unnecessary rapid acceleration and brakes.

It looks like there is no real need to restrict absolute value of acceleration and steering angle - it has no noticeable effect on the model.








