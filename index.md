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
Twenty experiments were conducted across nominal conditions, increased encoder noise, increased camera noise and a shifted obstacle. The strict ground truth verified docking success rates were 5/5, 5/5, 0/5, and 5/5, respectively. High camera noise exposed premature docking completion despite successful software termination, while low EKF ±2σ coverage revealed overconfident uncertainty estimates. The results demonstrate a working integrated pipeline under the tested nominal and selected disturbed conditions, while identifying specific limitations in estimation consistency and noise-sensitive docking decisions.

## System Design

The system includes sensing, state estimation, waypoint navigation, obstacle avoidance and visual docking.

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
