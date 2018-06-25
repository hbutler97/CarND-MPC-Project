# SDC Udacity MPC

---

The purpose of this project is to implement a model predictive controller (MPC) to drive a vehicle along a desired path



## Model in use

The model used for this project is a bicycle model that includes the following states:
* position `x`
* position `y`
* orientation `psi`
* velocity `v`
* cross-track error `cte`
* orientation error `epsi`
The control inputs are:  
* steering angle `delta`
* acceleration `a`

The moment of the model is given by the following equations:
```
// values at timestep [t+1] based on values at timestep [t] after dt seconds 
// Lf is the distance between the front of the vehicle and the center of gravity

x[t+1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt;
psi[t+1] = psi[t] + v[t]/Lf * delta[t] * dt;
v[t+1] = v[t] + a[t] * dt;
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt;
epsi[t+1] = psi[t] - psi_des + v[t]/Lf * delta[t] * dt;

```


## Timestep Length and Elapsed Duration (`N` & `dt`)

The prediction horizon `T` is the product of the timestep length `N` and elapsed duration `dt`.
Timestep length is the total number timesteps in the horizon and elasped duration is the amount of time between acutations.

N = 13 and dt = .1 yielded the best results

Many combinations of N and dt were tried and the general trend was the high the N, the more oscillations around the reference would occur.  Lower values of N, would cause the car drive straight off the track.



## Polynomial Fitting and MPC Preprocessing
As shown in the project overview, points were transformed to the vehicle's coordinate system and making that first point the origin.

Next, the orientation is also transformed to 0 so that the heading is straight forward. Each point is rotated by psi degrees.

After that the vector of points is converted to an Eigen vector so that it is ready to be an argument in the polyfit function where the points are fitted to a 3rd order polynomial. That polynomial is then evaluated using the polyeval function to calculate the cross-track error.

## Model Predictive Control with Latency

A delay of 100 ms need to be taken care of after the MPC works. When this latency was first introduced, the model oscillated about the reference trajectory.

Initial states were set after 100 ms. This allows the vehicle to "look ahead" and correct for where it will be in the future instead of where it is currently positioned.

double Lf = 2.67;
double dt = 0.1;

double pred_px = 0.0 + v * dt;
double pred_py = 0.0;
double pred_psi = 0.0 + v * -delta/Lf * dt;
double pred_v = v + a * dt;
double pred_cte = cte + v * sin(epsi) * dt;
double pred_epsi = epsi + pred_psi;

Eigen::VectorXd state(6);
//state << 0, 0, 0, v, cte, epsi;
state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;