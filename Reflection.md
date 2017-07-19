# CarND-Controls-MPC Reflection
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./mpc.png "mpc.png"
[image2]: ./latency.png "latency.png"
[image3]: ./no-latency.png "no-latency.png"

We have learnt three ways to control the car driving so far. The first way known as 'Behavior Cloning', which requires human driver to demonstrate how to drive in all scenarios and then we use a deep learning model to learn from the demonstration. The problem of this method is firstly the model requires quite a lot training data to learn. The generated model can only work for the trained scenarios as well. If the testing scenario is out of the training ones then the model will have no clue how to deal with it, and the quality of the training data is also very critical as well. If there's any human error in the training data then the model will repeat the error without hesitating. Considering there’re lots of road conditions, generating enough training data will be very difficult in practice. 

Then we learned the second controlling method -- so called "PID controller", which is a very mature and widely used controlling method. Simply speaking, it set up a reference line in the driving direction, measures the error of the car position against the reference line and minimizes the error. This method proved to be very effective. With this method, my car can drive at nearly 70mph, while using Behavior Cloning the car can only drive at 35mph approximately. But the PID controller is still not perfect. The PID controller can only react to existing errors. They don't have the ability to predict the route ahead and they can’t prevent the error from happening. As the consequence, when the car drives very fast, the PID controller will fail keeping the car on the track.

Then comes the last controlling method we just learned: the Model Predictive Controller, or MPC for short, which just as the name suggested, is able to minimize cost for the predicted route. With the power of MPC controller,  my can drive at 100mph, which is the upper limit of the simulator.

## MPC in detail
The model takes seven input for optimizing: the state of the vehicle <x,y,v>, the errors <cet,epsi>, and the controller controls the steering angle and throttle of the vehicle. 

![MPC][image1]

During each feedback loop, the algorithm constantly predicts the route according to the current state and re-evaluates the input state to find the optimal actuation and minimize a cost function. It predicts the next points and tries to find the optimal actuation using the so-called kinematic model, which is a simplified vehicle model. The solver also takes constraints as well as the cost function and finds the optimal actuation. 

The cost function plays key role in terms of tuning the model. By putting different weight on each parameter you can tell the model what’s the goal of the optimization.

Here’s my cost function and the coefficients:
```C++
    double cf_cte = 1.0;
    double cf_epsi = 5.0;//
    double cf_v = 1.0;
    double cf_delta = 10000.0;
    double cf_a = 1.0;
    double cf_dd = 100.0;
    double cf_da = 1.0;

	…
	    	    
    for (int i = 0; i < N; i++) {
      fg[0] += cf_cte * CppAD::pow(vars[cte_start + i], 2);
      fg[0] += cf_epsi * CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += cf_v * CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize actuator use
    for (int i = 0; i < N - 1; i++) {
    fg[0] += cf_delta * CppAD::pow(vars[delta_start + i], 2);    fg[0] += cf_a * CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize actuator change
    for (int i = 0; i < N - 2; i++) {
      fg[0] += cf_dd * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);   
      fg[0] += cf_da * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
```

## Time step Length and Elapsed Duration
The number of predicted steps N and the interval between steps dt describe the perdition for which the model looks ahead and predicts the impact of its actuation in the future. These parameters impact the computational complexity of the model and how it will respond to changes. Setting a very large N results in more computations and a slower model, but if the N was set too small or the dt is not enough, then the model will lost its prediction ability and the car experienced a zigzag effect.

The chosen values were 15 for N and 0.05 for dt. These values performed reasonably well for the given problem. 

## Polynomial Fitting and MPC Preprocessing
The overall process of fitting a polynomial to the track waypoints involved mapping between global and vehicle coordinates and vice versa. The waypoints are given in a global coordinate system. A polynomial can be fitted on those points; however, this introduces complexities in finding the correspondence with vehicle coordinates. A simpler solution is to map the global waypoint coordinates to vehicle coordinates, thus effectively turning the vehicle to the origin of reference. In doing so the position of the vehicle effectively becomes (0,0) and its orientation becomes 0. The coefficients of the fitted polynomial are found and the vehicle state is passed to the MPC. 

The actuators found by the MPC are given back to the simulator. A final pre-processing step involved normalizing the steering angle by dividing with rad(25) to shift the values between [-1 1].

## Latency
The latency between sending command to the car and the car really makes response can be an annoying problem otherwise but the MPC controller can deals with it extremely well.  My way of including the latency into the model is in each controlling loop, let the measured state move ahead a little bit according to the current throttle, speed, driving direction and the latency period setting, which is 0.1s in my calculation. 

Here’ s the car driving without handling latency.  Please notice that the car was driving at the outer side of the turning reference line.
![no_latency][image3]

And here’s the track with latency handling. The car can now predict the turning ahead and cuts into the inner side of the turning reference line. 
![latency][image2]

## Conclusion
The MPC model is a great enhancement over PID model. My car can even drives pretty smoothly at 100mph. But it still rolls onto the curb when doing sharp turns. So I slow down the speed to 80mph as the final commit.
