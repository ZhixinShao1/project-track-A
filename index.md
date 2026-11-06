---
layout: default
---
---
title: TurtleBot3 Visual Waypoint Navigation and Docking
---

**Track A**
**Platform: TurtleBot3 Waffle Pi (MuJoCo Simulation)**
**Group Number: 21**
**Team Members: Zhixin Shao (25550169)**

[Code Repository](https://github.com/ZhixinShao1/project-track-A) | 

## Abstract

This project develops and evaluates a visual waypoint navigation and autonomous docking system for a TurtleBot3 Waffle Pi robot in MuJoCo. Wheel encoders provide motion information for an EKF, and an RGB camera measures the range and bearing of an ArUco landmark to correct the estimated pose. A proportional feedback controller drives the robot towards a waypoint, a 2D LiDAR supports local obstacle avoidance and a visual docking controller aligns the robot with a charging station marker. All navigation decisions use estimated states or sensor measurements rather than simulator ground truth.

Twenty simulation trials were conducted under four conditions: nominal operation, increased encoder noise, increased camera noise and a shifted obstacle. The robot achieved all docking successes in each of the nominal, high encoder noise, and shifted obstacle conditions, giving 15 successful trials overall. These results demonstrate that sensing, EKF localisation, waypoint control, obstacle avoidance and visual docking work together under the tested conditions. In cases with high camera noise, the number of completely successful system runs was 0/5. However, camera noise testing and uncertainty analysis using the EKF have also provided a useful basis for further system improvements.

## 1. Introduction and System Overview

This project's objective is to integrate modelling, sensing, state estimation and feedback control into one reproducible mobile robot simulation. The TurtleBot3 starts near the world frame origin, navigates towards the waypoint at (1.35, 0.00) m, avoids a room obstacle, and then approaches an ArUco marker fixed at (1.80, 0.00) m. The desired final distance from the robot base to the marker is 0.34 m, The objective is to integrate modelling, sensing, state estimation, and feedback control into one reproducible mobile-robot simulation. The TurtleBot3 starts near the world frame origin, navigates towards the waypoint at (1.35, 0.00) m, avoids a room obstacle, and then approaches an ArUco marker fixed at (1.80, 0.00) m. The desired final distance from the robot base to the marker is 0.34 m, subject to a distance tolerance of 0.015 m and a bearing tolerance of 7°.

The system uses four connected stages. Firstly, wheel encoder and camera observations are collected. Secondly, the EKF predicts the robot pose using odometry and corrects it when a valid marker observation is available. Thirdly, waypoint feedback generates linear and angular velocity commands, LiDAR measurements activating a temporary detour when necessary. Finally, visual docking uses the marker's relative range and bearing for accurate alignment and approach. A state machine manages transitions between navigation, avoidance and docking.

In this project, encoders support continuous prediction, camera observations limit odometry drift, LiDAR provides obstacle information and the controllers turn state estimates into robot motion. MuJoCo ground truth is not supplied to the feedback controller. It is used only for plots, error metrics, collision evaluation where implemented, and final success verification. 


                              Figure 1. Integrated sensing, estimation and feedback control pipeline.

## Experimental Results

The system was evaluated across four conditions with five trials per condition.

| Condition | Success Rate |

| Nominal | 100% |
| High Encoder Noise | 100% |
| High Camera Noise | 0% |
| Shifted Obstacle | 100% |

## Demo Video

To be added.

## Team Contributions

To be added.

## Generative AI Declaration

To be completed with the actual tools and purposes.
