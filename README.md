# Model Predictive Control (MPC)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---

This project builds on the [base code](https://github.com/udacity/CarND-MPC-Project. Details on dependencies and set-up can be found there.
After cloning the project, it can be compiled and run ttyping below in the command line e.g. bash on Ubunto on Windows
``` 
mkdir build && cd build
cmake .. && make
./mpc
```

The code is based on a kinematic model of the car's movement. Thereby, the input values from the car are used to predict the best car controls to stay on track.

### Actuator (control vector)
The car will be controlled using two variables, combined in the actuator 2-Vector: actuator = [delta, a].
The actuator is the input to the car where delta is the steering angle between -25 and 25 degree and a the acceleration normalized to [-1,1].
The steering angle delta is positive for right turns and negative for left turns. Note that this is opposite as in the class room.


### State and motion model
The state of the car is described by a 6-Vector: state = [x, y, psi, v, cte, epsi]

| Variable | Description | Motion model (after dt) |
|:--------:|:-----------:|:------------------------:|
|x| position | x1 = x + v * cos(psi) * dt |
|y| position | y1 = y + v * sin(psi) * dt |
|psi| orientation | psi1 = psi + v * delta/Lf * dt |
|v| velocity | v1 = v + a * dt |
|cte = f(x) - y| cross track error : distance to modeled trajectory f(x) | cte1 = cte + v * sin(epsi) * dt |
|epsi = psi - arctan(f'(x))| difference in orientation | epsi1 = epsi + v * delta/Lf * dt |

The simulator returns x, y, psi and v and a couple of more parameters. [DATA.md](./DATA.md) contains a description of the data sent back from the simulator.

### Trajectory
The road is modeled by a 3rd-order polynomial f(x) using the center-line-points ptsx and ptsy.
These points are first transfered in the local coordinate system by the car.
The coordinate system is rotated and translated such that the origin is in the center of mass of the car and the x-axis is along the car's orientation.

The cte and epsi are calculated relative to this ideal trajectory.

### Optimization / Cost function
The next values for the actuator are found by minimizing the cost function fg(state, actuator)

fg = term_diff + term_actuator + term_change

* term_diff: difference to trajectory (cte, psi, v); 3 terms
* term_actuator: high control values; 3 terms
* term_change: high changes between two times; 2 terms

The weights of each term was tuned until the car run soomthly through the track.

After normalisation, the actuator values are send to the simulator/car.

### Parameterization
The amount of timesteps N (or total time T = N*dt) and the length of every timestep dt was varied to find the optimal trade-off between computation cost and precision.
The computation time increases with increasing N and decreasing dt.

The selected values for the timesteps are

| N | dt |
|:---:|:---:|
| 20 | 0.1 |

### Latency
In the model, a 100 ms latency was introduced which simulates real behaviour better.
In code this is handled by updating the car's state using the same motion model before the cost funtion is minimised. However note that due to the coordinate transformation, x, y and psi are zero and above equations simplify.
