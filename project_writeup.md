# Model Predictive Control Project

The goal of this project was to implement an MPC controller to control the steer angle and throttle of a simulated car as it drives around a track.

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Compilation

#### Your code should compile.

The code compiles using cmake and make commands as described in project instructions.

### Implementation

#### The Model

*Student describes their model in detail. This includes the state, actuators and update equations.*

I have implemented a Model Predictive Controller using a kinematic model with the following state vector:

State of vehicle:

* x - x-position
* y - y-position
* ψ - heading
* v - velocity

State error:

* cte - cross track error
* eψ - heading error

Actuation:

* δ - steer angle
* a - throttle (acceleration)

The following update equations were used to constrain the model prediction optimisation:

x(​t+1)​​ = x(​t) + v(​t) ∗ cos(ψ(​t)) ∗ dt

y(t+1) = y(​t)​​ + v(​t) ∗ sin(ψ(​t)) ∗ dt

ψ(​t+1)​​ = ψ(​t) + (v(t) / L​f) *​ δ(t) * dt

v(​t+1) = v(t) + a(t) * dt

cte(t+1) = cte(t) + v(t) * sin(eψ(​t)) * dt

eψ(​t+1) = eψ(​t) + (v(t) / Lf) * δ(t) * dt

And the following actuator constraints were used:

δ ∈ [-25°, 25°]

a ∈ [-1, 1]

I added cost components to cover the following aspects of the vehicle motion/actuation:

* Cross Track Error - drive near the centre of the road
* Heading Error - drive in the same direction as the road
* Velocity Error - drive near the specified velocity
* Steer Actuation - steer as little as possible
* Throttle Actuation - use throttle as little as possible
* Steer Rate - don't change steer angle quickly (smooth cornering)
* Throttle Rate - don't change acceleration quickly (smooth acceleration)
* Fast Turn - don't turn sharply at high speed

Each of these components was tuned individually to apply different weightings to each element of the cost. This allowed me to tune the trajectory that the MPC came up with.

#### Timestep Length and Elapsed Duration (N & dt)

*Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

I used values of N=10 and dt=0.1 for my MPC. I had started out with N=25 and dt=0.05 but I found that I could actually get away with reducing the horizon (T) to 1 second and increasing the step size to 100ms. This reduced the computation time for the controller significantly.

#### Polynomial Fitting and MPC Preprocessing

*A polynomial is fitted to waypoints.*

*If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

Before processing the map-frame waypoints I first transformed them into the vehicle reference frame. I then used polyfit to fit a 3rd order polynomial to the track waypoints which gave me a reference function to calculate the state errors against in my trajectory prediction. 

#### Model Predictive Control with Latency

*The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

In order to deal with the latency of the vehicle actuation I predicted the state of the vehicle 100ms in the future from the current state. This meant that I was optimising the actuation for the future vehicle state (at the time that the actuation would actually be applied to the vehicle). I used the same vehicle model to predict the state as for the MPC optimisation.

### Simulation

#### The vehicle must successfully drive a lap around the track.

*No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).*

*The car can't go over the curb, but, driving on the lines before the curb is ok.*

Here is a [video](./project_output.mp4) of the car driving around the track at a speed of around 80 units.
