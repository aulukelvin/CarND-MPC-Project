# CarND-Controls-MPC Reflection
Self-Driving Car Engineer Nanodegree Program
[//]: # (Image References)
[image1]: ./mpc.png "mpc.png"
[image2]: ./latency.png "latency.png"
[image3]: ./no-latency.png "no-latency.png"

We have learnt three ways to control the car driving so far: the first way is known as 'Behaviour Cloning', which requires human driver to demonstrate how to drive in all scenarios and then we use a deep learning model to learn from the demonstration. The problem of this method is not so satisfying because firstly the model requires quite a lot training data to learn. If the road condition is non-trivial then generating training data will be very difficult. The model is learning from the training data so if there's any human error then the model will repeat the error without hesitating. And the generated model can only work for the trained scenarios as well. If the testing scenario is out of the training ones then the model will have no clue how to deal with it.

Then we learned the second controlling method -- so called "PID controller", which is very mature and widely used controlling method. Simply speaking, it set up a reference line in the driving direction, measure the error of the car position against the reference line and minimize the error. This method proved to be very effective. With this method, my car can drive at nearly 70mph, while using Behaviour Cloning the car can only drive at 35mph approximately. But the PID controller is still not good enough because the PID controller can only react to existing errors. They don't have the ability to predict or prevent the error from happening. As the consequence, when the car drives very fast, the PID controller will fail to keep the car on the track.

Then comes the last controlling method we just learned: the Model Predictive Controller, or MPC for short, which is trying to minimize cost for the route ahead.

## MPC in detail
The implemented MPC model drives the car around the track in the simulator. The model takes as input the state of the vehicle <x,y,v> and the errors <cet,epsi>. The controller controls the steering angle and throttle of the vehicle. The algorithm constantly re-evaluates given the input state to find the optimal actuation and minimize a cost function. It predicts the next points and tries to find the optimal actuation using the above kinematic model. The solver also takes constraints as well as the cost function and finds the optimal actuation.
![MPC][image1]

## Timestep Length and Elapsed Duration
The parameters N and dt describe the perdition horizon for which the model looks ahead and predicts the impact of its actuation in the future. These parameters impact the computational complexity of the model and how it will respond to changes. Setting a very large N results in more computations and a slower model. The chosen values were 10 for N and 0.1 for dt. These values performed reasonably well for the given problem. In addition, when not considering the latency different values can also work well. Some of the values that I tried were N=25,dt=0.2 and N=10,dt=0.01; the latter values do not allow the vehicle to predict too far into the future which in non-ideal cases is problematic. For example, in sharp turns in the simulation, the car experienced a zig-zag effect.

## Polynomial Fitting and MPC Preprocessing
The overall process of fitting a polynomial to the track waypoints involved mapping between global and vehicle coordinates and vice versa. The waypoints are given in a global coordinate system. A polynomial can be fitted on those points, however, this introduces complexities in finding the correspondence with vehicle coordinates. A simpler solution is to map the global waypoint coordinates to vehicle coordinates, thus effectively turning the vehicle to the origin of reference. In doing so the position of the vehicle effectively becomes (0,0) and its orientation becomes 0. The coefficients of the fitted polynomial are found and the vehicle state is passed to the MPC. The MPC finds actuator values based on the following cost function with weighted objectives:

The actuators found by the MPC are given back to the simulator. A final pre-processing step involved normalizing the steering angle by dividing with rad(25) to shift the values between [-1 1].

## Latency
The way that I solved the latency problem was to set the dt to be equivalent to the latency time. In doing so the model was able to predict the next state in the time needed for the actuation to be applied. N was also kept reasonably low for computation efficiency. In the end, N was set to 10 and dt to 0.1. Interestingly, the car was also able to drive the track with parameters N=5 and dt=0.05, However, it was reckless and in some cases wondered outside the track before going back in again.
![no_latency][image3]

![latency][image2]
