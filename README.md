# Cable-Tension-Estimate

## Overview

This project aims to estimate the cable tension acting on a quadrotor by constructing a physical system consisting of a drone, spring, and payload. Instead of using direct force sensors, we infer tension through spring deformation, allowing for a lightweight and cost-effective solution.

### System Architecture
- Platform : Crazyflie 2.1 nano quadrotor + Thrust Upgrade Bundle


---







# Cable Tension Estimate Experiment Data

## 1. Spring Test

### 1.1 - Single Crazyflie Test

#### 1.1.1 - Crazyflie Hover Test (Spring)

| Payload (g) | Task Completion flag |
|-------------|----------|
| 9           | O     | 
| 10          | O     | 
| 11          | O     | 
| 12          | O     | 
| 13          | O     | 
| 14          | O     | 
| 15          | O     | 
| 16          | O     | 
| 17          | O     | 
| 18          | O     | 
| 19          | O     | 

#### 1.1.2 - Crazyflie Sqaure Test (Spring)

| Payload (g) | Task Completion flag |
|-------------|----------|
| 9           | O     | 
| 10          | O     | 
| 11          | O     | 
| 12          | O     | 
| 13          | O     | 
| 14          | O     | 
| 15          | O     | 
| 16          | O     | 
| 17          | O     | 
| 18          | O     | 
| 19          | O     | 

#### 1.1.3 - Crazyflie Circle Test (Spring)

| Payload (g) | Task Completion flag |
|-------------|----------|
| 9           | O     | 
| 10          | O     | 
| 11          | O     | 
| 12          | O     | 
| 13          | O     | 
| 14          | O     | 
| 15          | O     |
| 16          | O     |
| 17          | O     |
| 18          | O     |
| 19          | O     |

#### 1.1.4 - Crazyflie Updown Test (Spring) 

| Payload (g) | Task Completion flag |
|-------------|----------| Test Date |
| 9           | O     | 2025.04.21 |
| 10          | O     | 
| 11          | O     | 
| 12          | O     | 
| 13          | O     | 
| 14          | O     | 
| 15          | O     | 
| 16          | O     | 
| 17          | O     | 
| 18          | O     | 
| 19          | O     | 

### 1.2 - 2 Crazyflie Test

#### 1.2.1 - 2 Crazyflie Hover Test (Spring)

| Payload (g) | CF1 UP | CF1 DOWN | CF4 UP | CF4 DOWN |
|-------------|--------|----------|--------|----------|
| 10          | 1.23   | 1.21     | 1.18   | 1.20     |

#### 1.2.2 - 2 Crazyflie Updown Test (Spring)

| Payload (g) | CF1 UP | CF1 DOWN | CF4 UP | CF4 DOWN |
|-------------|--------|----------|--------|----------|
| 10          | 1.23   | 1.21     | 1.18   | 1.20     |

## 2. Cable Test

### 2.1 - Single Crazyflie Test

#### 2.1.1 - Crazyflie Hover Test (Cable)

| Payload (g) | CF1 UP | CF1 DOWN | CF4 UP | CF4 DOWN |
|-------------|--------|----------|--------|----------|
| 10          | 1.23   | 1.21     | 1.18   | 1.20     |

#### 2.1.2 - Crazyflie Updown Test (Cable)

| Payload (g) | CF1 UP | CF1 DOWN | CF4 UP | CF4 DOWN |
|-------------|--------|----------|--------|----------|
| 10          | 1.23   | 1.21     | 1.18   | 1.20     |

