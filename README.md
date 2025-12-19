# Spring-Tension-Estimate

## Overview

Experimental data and code for spring tension estimation using a dual-multicopter system with a suspended payload of 20–30 g.

Experiment content : Hovering, Updown, Circle Test

### System Architecture

### Thrust-PWM correction alpha from Dual Multicopter Flight.

1. Suspend a payload (10–20 g) from 2 Crazyflie using a spring and perform hovering flight.
2. Record all ROS topics during flight, including Crazyflies position, payload position, PWM, and mass * acceleration inertial terms:
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


아래 내용은 README.md의 #### Processing Pipeline 섹션에 그대로 복사-붙여넣기 가능한 형식으로 작성했습니다.
앞에서 정리한 Spring Tension Estimation 절차와 정확히 대응되도록 구성했습니다.

#### Processing Pipeline

1. A payload of 20–30 g is suspended using springs from a dual-multicopter system, and flight experiments are conducted under hovering, circular, and ascending–descending maneuvers. Each experiment is repeated three times.
2. During flight, the Crazyflie positions, payload position, thrust (\(f_t\)), and spring tension (\(f_s\)) are recorded into ROS bag files using:
   - `rosbag record -a`
3. The recorded data are processed to compute the predicted spring tension and the measured spring tension using the following pipeline:
   - `point_index.py -> spring.py -> timestamp.py -> Circle_RMSE.py or Hover_RMSE.py or Updown_RMSE.py`
4. The estimation performance is evaluated by recording the RMSE, RMSE norm, and normalized RMSE.

#### `point_index.py`  
- Identifies the payload as a point mass from the motion-capture data and extracts its position as a single reference point.  
- This payload position is used as the ground-truth input for spring length and tension computation.

#### `spring.py`  
- Computes the **measured spring tension** based on the relative positions of each Crazyflie and the payload.  
- Using a Hooke’s law–based spring model,
  \[
  F_s = k (L - L_0) + b,
  \]
- the script estimates the spring force vector and magnitude for each multicopter.  
- The resulting spring tension (vector and norm, in gf) is appended to the original ROS bag as new topics.

#### `timestamp.py`  
- Converts absolute ROS timestamps to **relative time starting from 0 s**, ensuring consistent temporal alignment across multiple trials and flight scenarios.  
- This step enables unified time-window selection for RMSE evaluation.

#### `Circle_RMSE.py`, `Hover_RMSE.py`, `Updown_RMSE.py`  
- Evaluate the accuracy of the spring tension estimation for different flight modes.  
- For each experiment:
  - predicted spring tension (from onboard logs) and  
  - measured spring tension (from the spring-payload model) are synchronized in time and compared.  
- The scripts compute RMSE for each axis, the tension-norm RMSE, and the normalized RMSE, and generate comparison plots.y'
