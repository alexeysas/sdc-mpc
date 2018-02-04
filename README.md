# MPC (Model Predictive Control)
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./images/1.png "Model"
[image2]: ./images/2.png "Model"

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

Also, we are getting trajectory in the global coordinates, however it is beneficial to convert trajectory to the vehicle coordinates (where vehicle coordinates are (0,0) and orientation along X axes - ψ = 0). This simplifies error equzions to the simple equazions below:

![alt text][image2]



## Paramertes Tuning

## Latency 



