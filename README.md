# Spring-Tension-Estimate

## Overview

Experimental data and code for spring tension estimation using a dual-multicopter system with a suspended payload of 20–30 g.

Experiment content : Hovering, Updown, Circle Test

### System Architecture

### Thrust-PWM correction alpha from Dual Multicopter Flight.

1. Suspend a payload (10–20 g) from 2 Crazyflie using a spring and perform hovering flight.
2. Record all ROS topics during flight, including Crazyflie position, payload position, PWM, and mass * acceleration inertial terms:
   - `rosbag record -a`
3. Process the recorded data to compute PWM and thrust using the following pipeline:
   - `point_index.py -> thrust.py -> timestamp.py -> average.py`
4. Perform quadratic regression between PWM and thrust using:
   - `regression.py`

#### Processing Pipeline

##### `point_index.py`
- Identifies and separates the payload as a point mass from the motion-capture data.

##### `thrust.py`
- Automatically processes all `alpha_index9.bag` files in the current directory.
-  Estimates the spring tension and thrust for each Crazyflie using ROS bag data.  
  Specifically, it:
  - synchronizes payload position, Crazyflie pose, and onboard inertial log data,
  - computes the spring force using a Hooke’s law–based model,
  - transforms forces between the world and body frames, and
  - reconstructs the body-frame thrust using inertial, gravity, and spring force terms.  
  The estimated forces are written to a new ROS bag file for further analysis.

#### `timestamp.py`  
- Normalizes ROS timestamps so that the experiment start time is aligned to 0 seconds, enabling consistent time-window selection across different experiments and bag files.

#### `average.py`  
- Computes time-averaged values of the estimated vertical thrust and PWM commands over a specified steady-state time window. The averaged results are used to construct paired PWM–thrust data points for regression.

#### `regression.py`  
- Applies a quadratic regression model to identify the PWM–thrust relationship. The script also allows adjustment of the constant term to compensate for systematic bias observed in dual-multicopter flight experiments.


### Spring Tension Estimation.



