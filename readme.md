# Robotics: PID and Fuzzy Logic Controllers

## Overview
This project explores the implementation of PID (Proportional-Integral-Derivative) and Fuzzy Logic controllers in robotics. We focus on two key behaviors: right edge following and obstacle avoidance. The project demonstrates the advantages of Fuzzy Logic over traditional PID control in complex robotic navigation tasks.

## Table of Contents
1. [Introduction](#introduction)
2. [PID Controller](#pid-controller)
3. [Fuzzy Logic Controller](#fuzzy-logic-controller)
4. [Hierarchical Fuzzy Logic](#Hierarchical-Fuzzy-Logic)
5. [Results](#Results)

## Introduction
PID control is widely used in industrial processes due to its simple structure and clear physical meanings. However, it has limitations in complex systems and noisy environments. Fuzzy logic, developed by Lotfi A. Zadeh, offers an alternative approach that mimics human thinking by reasoning approximately rather than precisely.

In this project, we implement and compare PID and Fuzzy Logic controllers for robotic navigation tasks.

## PID Controller
We implemented a PID controller for right edge following. The process involved:

1. Sensor Selection: LiDAR (Light Detection and Ranging)
2. Desired Distance Determination: Target distance of 0.6 units from the wall
3. PID Parameter Tuning: Adjusting Kp (proportional gain), Ki (integral gain), and Kd (derivative gain)
4. Error Calculation: Using the formula `error = desired distance - current distance`
5. Speed Setting: Constant linear speed of 0.1, with angular speed calculated based on PID output

While the PID controller performed well for right edge following, it showed limitations in handling complex scenarios and noisy environments.

## Fuzzy Logic Controller
We implemented Fuzzy Logic Controllers for both right edge following and obstacle avoidance.

### Right Edge Following
1. Sensor Selection: Front Right and Back Right sensors
2. Linguistic Variables: 
   - Inputs (distance): Close, Medium, Far
   - Outputs: Linear speed (Slow, Medium, Fast), Angular speed (Front, Left, Right)
3. Membership Functions: Defined for each linguistic variable
4. Rule Base: Mapping between inputs and outputs (see Table 1 in project presentation)
5. Fuzzification, Inference, and Defuzzification processes implemented

### Obstacle Avoidance
Similar process to right edge following, with adjusted rule base (see Table 2 in project presentation)

## Hierarchical Fuzzy Logic
We integrated the right edge following and obstacle avoidance behaviors into a single system. The integration process involved:

1. Combining sensor inputs from both behaviors
2. Developing a unified rule base
3. Implementing a context layer for behavior switching

## Results
We have created video demonstrations for each component of the project:

1. [PID Right Edge Following](https://youtu.be/LcpSJCoUrcE)
2. [Fuzzy Logic Right Edge Following](https://youtu.be/Qdcx1GOljIQ)
3. [Fuzzy Logic Obstacle Avoidance](https://youtu.be/8zcmklsf8PU)
4. [Integrated Behavior System](https://youtu.be/BE4WXoKOD4A)

These videos showcase the performance of each controller and the integrated system in various scenarios.

## Conclusion
This project demonstrates the advantages of Fuzzy Logic Controllers over traditional PID controllers in complex robotic navigation tasks. The Fuzzy Logic approach showed better performance in handling uncertainty and integrating multiple behaviors.

---

Project by Ahmed jafar Osman Ahmed

For more detailed information, including MATLAB simulations, membership function graphs, and complete rule bases, please refer to the full project presentation and code in this repository.