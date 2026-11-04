# project-track-A
Track A Mobile Robot Visual Waypoint Navigation
Platform: TurtleBot3 Waffle Pi (MuJoCo Simulation)

This project develops an autonomous navigation and visual docking system for a TurtleBot3 robot using MuJoCo simulation.

The robot uses wheel encoders and a camera for EKF based localisation, a state feedback controller for waypoint navigation and a 2D LiDAR for obstacle avoidance. It then uses ArUco marker detection to approach a docking station.

The complete simulation includes:

State space modelling and EKF localisation
Waypoint navigation and LiDAR obstacle avoidance
ArUco based visual docking
Quantitative evaluation and robustness testing