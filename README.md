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





