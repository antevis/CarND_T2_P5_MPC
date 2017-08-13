[//]: # (Image References)
[formulae]: ./img/kinematic_equations_small.png
[latencyFormulae]: ./img/latency_equations_small.png
[latencyChart]: ./img/chart.png

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model

For this project, I follow the **Global Kinematic Model**. It may be useful to
distinguish three significant parts within it: *State*, *Actuators*, *Equations*.

#### State

The vehicle's state consists of the following elements:

* x - *x coordinate*
* y - *y coordinate*
* psi - *orientation (yaw)*
* v - *velocity*
* cte - *cross-track error*
* epsi - *yaw error*

For convenience, it has been implemented as a dedicated struct in MPC.h as follows: 

```cpp
template <typename T>
struct State {
    T x;
    T y;
    T psi;
    T v;
    T cte;
    T epsi;
};
```
Generics are useful here, since values may be both `CppADD::AD<double>` and `double`.
In fact, my implementation implies coordinate conversion to vehicle's coordinate system
with `x, y, psi` to be always equal to `0`, but since I am going to take ***latency*** into
account, those elements will eventually have non-zero values.

#### Actuators

The model has two actuators: **steering** and **throttle**, and they play a dual role.
 
1. We use actuators' values provided by the Simulator environment at each telemetry package to 
estimate vehicle's state with **latency**.

    **Steering** angles span the interval of [-25,+25] degrees, **wherein values are in radians**, 
    with negative/positive values representing angles to the left/right of the zero point respectively.

    **Throttle** works the same way within the interval of [-1,+1], where `-/+` represent 
   `breaking/acceleration`.

2. The Model's solution **primary objective** is to compute the new actuators values and return them to the 
Simulator environment as the control commands for the vehicle.

#### Equations

The following standard equations have been adopted and implemented for the Kinematic model:

![alt text][formulae]

However, to reflect **latency**, I am using slightly more sophisticated calculations.
I decided not to neglect the ***lateral displacement*** (which is the `dy` component in the vehicle's
coordinate system), and compute the vehicle's state at the end of the latency delay as follows:

![alt_text][latencyFormulae]

Where:

* `phi` - vehicle's orientation at time `dt`, given the steering angle `delta` at time `0`.
* `L` - path length traveled by the vehicle during `dt`
* `R` - curvature radius of `L`
* `dx` - longitudinal displacement during `dt`
* `dy` - lateral displacement during `dt`
* `v_dt` - vehicle's velocity at time `dt`

Here is the chart supporting the above calculations:

![alt_text][latencyChart]

I assume that given the initial `delta` as the vehicle's steering angle, it moves along the segment of a 
circle of radius `R` for a distance of `L`. To compute `dx`, the absolute value of `psi` being used,
as `dx` assumed to be always positive, regardless of the `psi`'s sign.

### MPC solution

In this project, the Model Predictive Control has been implemented using the 
[**Ipopt**](http://projects.coin-or.org/Ipopt), which is  a library for 
large-scale nonlinear optimization.

Specifically, `CppAD::ipopt::solve<Dvector, FG_eval>` -- is the method that does most of the work.

It consumes long vectors of all the independent variables, variables bounds (if any) and constraints 
defined by the vehicle model, and returns locally optimal values of the same dimensionality as the vector
of independent variables.

#### Choosing the number of timesteps and `dt`
The vectors' dimensionality directly **affects** the consequent **computational complexity**. 
For `N` timesteps of observations, there are `N - 1` controls actuations. With the dimensionality of `6` for
observation (`state`) and `2` for actuation (`steer, accel`), the total length of the variables vector
will be:
 
 `25 * 6 + (25-1) * 2`, which is `198` elements.
 
With the kinematics of the vehicle, for the MPC control we aren't particularly interested in what's going 
to happen beyond the 1 second time horizon at each timestep, so picking `10` for the number of 
timesteps `N` and `100` ms for `dt` seems fairly reasonable and produces the variables vector with the 
length of `78`.

#### The code

The solution is largely based on the 
[implementation of the MPC given at classes](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp).
Though, I have completely torn it apart and deeply refactored to my flavor. The final solution might 
seem a bit over-engineered but doing so really helped to understand what's actually going on under the
hood and makes the solution easily scalable. The most significant conceptual alteration is that I completely removed 
the `FG_eval` class and transferred it's single-method  functionality to the `MPC` class itself, using it 
as a delegate for `CppAD::ipopt::solve<Dvector, FG_eval>`:

```cpp
CppAD::ipopt::solve<Dvector, MPC>(options, vars, vars_lowerbound,
                                  vars_upperbound, constraints_lowerbound,
                                  constraints_upperbound, *this, solution);
```

#### Cost function

A lot of experimentation to the cost function have been undertaken. As a result, I came up with a couple of 
additional penalties for co-dependence of current speed, acceleration and cte/epsi:

```cpp
/// Adds additional cost for co-dependence of curent speed, acceleration and cte/epsi
fg[0] += cost_weights[5] * CppAD::pow(curr_accel * curr_v * curr_cte, 2);
fg[0] += cost_weights[6] * CppAD::pow(curr_accel * curr_v * curr_epsi, 2);
```

The idea is that the the faster the vehicle moves, the greater concern its `cte` and `epsi` pose. And the
greater the errors, the less acceleration should be applied.

The experimentation showed that the `epsi`, which is the vehicle's orientation error, has much greater
(in fact, orders of magnitude greater) importance for the vehicle's behavior than the `cte`. 
Indeed, `cte` itself might not be much of a problem unless it's still within the lane bounds and the 
vehicle's orientation (`psi`) is close to the expected - it would just mean that the vehicle moves 
parallel to the expected trajectory with a slight lateral shift. While large `epsi` signifies that the vehicle 
isn't really parallel to the expected trajectory, which is obviously a great concern, especially at 
higher velocities.

### Basic Build Instructions

1. `mkdir build && cd build`
2. `cmake .. && make`
3. `./mpc`.
