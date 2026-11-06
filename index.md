---
layout: default
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


*Figure 1. Integrated sensing, estimtion and feedback control pipeline.*

## 2. State Space Modelling

### 2.1 Robot Motion Model

The TurtleBot3 is modelled as a planar differential drive robot with state x = [x, y, θ]ᵀ and control input u = [v, ω]ᵀ. Here x and y represent world frame position, θ is heading, v and ω are forward and angular velocities. Assuming planar motion and negligible wheel slip, the nonlinear equations are:

$$
\dot{x}=v\cos\theta,\qquad
\dot{y}=v\sin\theta,\qquad
\dot{\theta}=\omega
$$

The configured wheel radius is 0.033 m and wheel separation is 0.287 m. Differential drive kinematics convert left and right wheel angular velocities into v and ω. A discrete version of the model advances the EKF state estimate at each simulation step. This model is not exact because encoder noise, the simulation time step and differences between the ideal model and MuJoCo movement can cause errors.

### 2.2 Linearisation and Controllability

To check the state space model, it was linearised at a forward speed of 0.10 m/s and a heading of 0 rad. The reference speed of 0.10 m/s was chosen because it is nonzero and within the robot's normal speed range. When the robot moves forward, turning also changes its sideways position. A nonzero speed makes this relationship appear in the linearised model. 

The resulting continuous time matrices are:

$$
A=\begin{bmatrix}0&0&0\\0&0&0.10\\0&0&0\end{bmatrix},\qquad
B=\begin{bmatrix}1&0\\0&0\\0&1\end{bmatrix}.
$$

The controllability matrix was formed using B, AB, and A²B, gives a rank of 3, which matches the three robot states. This means the robot can control all three states near the selected operating point. Also it cannot move directly sideways, but it can turn and move forward to change its sideways position. 

### 2.3 Observability

The camera measures the distance and direction to an ArUco marker whose position is known. These two measurements can be predicted from the robot position and heading. For a marker at (xL, yL), the measurement model is:

$$
h(\mathbf{x})=\begin{bmatrix}\sqrt{(x_L-x)^2+(y_L-y)^2}\\
\operatorname{atan2}(y_L-y,x_L-x)-\theta\end{bmatrix}.
$$

The measurement Jacobian H was calculated from this model and used with the motion matrix to check observability. With the marker at (1.80, 0.00) m, it gives a rank of 3 at the selected operating point. This means the model can estimate all three states locally when the robot moves and receives valid marker measurements. Because one camera observation measures only distance and direction, so the EKF also uses the robot motion over time. If the marker is not visible, the EKF continues predicting from encoder data until the camera detects it again.

## 3. Sensing and Coordinate Frames

### 3.1 Sensor Configuration

The robot has three main sensors. Wheel encoders measure wheel movement and provide speed estimates for the EKF. The 2D LiDAR uses 72 rays, measures up to 3.5 m, and updates about every 0.05 s. It detects obstacles and helps the robot decide when to turn or stop. The RGB camera has a resolution of 320 × 240 pixels and a vertical field of view of 60°. OpenCV detects ArUco marker ID 0, with a side length of 0.18 m, and calculates its position relative to the camera.

The sensor settings were taken from the simulated robot and environment. The LiDAR provides obstacle distances in the tested room and the known marker size allows OpenCV to estimate its pose. When marker detection fails, the program does not use that measurement to update the EKF.

### 3.2 Camera Calibration and Coordinate Transformation

The camera uses a pinhole model. Its focal length was calculated from the image height of 240 pixels and the vertical field of view of 60°. Assuming equal focal lengths in both directions gives fx = fy = 207.85 pixels. The image centre is about (159.5, 119.5) pixels, and lens distortion is set to zero. 

The program uses four coordinate frames: W for positions in the room, B for the robot base, C for the camera and L for the ArUco marker. The camera is mounted 0.10 m ahead of the robot centre, so a distance measured from the camera is not the same as the distance from the robot base. After expressing the camera measurement in the forward or left directions, the program adds the camera offset:

$$
p_x^B=0.10+r_C\cos\beta_C,\qquad p_y^B=r_C\sin\beta_C.
$$

The program then calculates the distance and direction from the robot base to the marker. These are the measurements used by the EKF and docking controller. Without this correction, the 0.10 m camera offset could cause a large error compared with the allowed docking distance error of 0.015 m. In normal trials, no extra camera noise is added, but the EKF still assumes that measurements have some uncertainty. Other trials increase sensor noise to test the system.


*Figure 2. Frame definitions and camera to base measurement conversion.*

## 4. EKF-Based State Estimation

### 4.1 EKF Design

Wheel encoder errors can build up over time, so encoder data alone may give an inaccurate position. The EKF combines encoder based motion prediction with camera measurements whenever the ArUco marker is visible. It estimates x, y and θ and keeps a 3 × 3 covariance matrix P to describe uncertainty.

During prediction, the EKF uses the speed from the encoders to estimate where the robot will move next. The Jacobian F updates its uncertainty, while Q represents possible prediction errors. During correction, the EKF compares the camera measured distance and direction with the values expected from the estimated position. The Jacobian H measurements covariance R, and Kalman gain determine how much the estimate should change. The program also handles angle wraparound at +π and -π. If the camera cannot see the marker, the EKF skips correction and continues prediction.

An EKF was chosen because the robot motion and the camera bearing measurement are nonlinear. The EKF can use these models and estimat how uncertain the robot position and heading are.

### 4.2 Parameter Selection and Justification

The initial position standard deviations are 0.02 m for x and y, and the initial heading standard deviation is 0.035 rad. These values show that the robot starts at a mostly known position, but some uncertainty is allowed. For the process model, the standard deviations are 0.01 m, 0.01 m and 0.02 rad. For camera distance and bearing, they are 0.02 m and 0.035 rad. The program squares these values to form the diagonal elements of P₀, Q, and R. It also scales the process noise term by the simulation time step.

These values were selected as starting settings for the simulation. The aim was to balance encoder based prediction and camera based correction. Their performance was evaluated through simulation experiments under different sensor noise conditions. A larger Q tells the EKF to trust its motion prediction less. A smaller R tells it to trust the camera more. This helps when camera measurements are accurate but can cause bigger errors when they are noisy. The robot completed its main tasks with these settings, although the coverage results show that the covariance values could be improved.

### 4.3 Estimation Performance

The EKF estimates were compared with the true states recorded by MuJoCo. Position RMSE measures average position error and heading RMSE measures average angle error. ±2σ coverage is the percentage of time samples in which the true value falls within two estimated standard deviations of the EKF result.

In the five normal trials, mean position RMSE was 0.1078 m and mean heading RMSE was 0.3338 rad. The robot successfully navigated, avoided the obstacle and docked in all five trials using these estimates. This shows that combining encoder and camera data gave the controller enough information to complete the task.

The ±2σ coverage rates were 48.79% for x, 33.74% for y, and 27.06% for heading. For an ideal Gaussian model, the reference value is about 95.45%. Although the measured values are lower, the EKF still provided useful state estimates for navigation.


*Figure 3. State estimation errors and uncertainty intervals from an actual simulation run.*

## 5. Feedback Control and Visual Docking

### 5.1 Waypoint Navigation

The robot uses a proportional feedback controller. It calculates the distance d to the waypoint at (1.35, 0.00) m and the angle error α between its estimated heading and the waypoint direction. It then calculates the forward and turning speeds using:

$$
v=\operatorname{clip}(2.0d\max(0,\cos\alpha),0,0.16),\qquad
\omega=\operatorname{clip}(1.5\alpha,-0.80,0.80).
$$

The distance gain 2.0 makes the robot move faster when it is farther from the waypoint. The heading gain 1.5 makes it turn towards the waypoint. The cosine term reduces forward speed when the robot is facing the wrong direction. The 'clip' function limits forward speed to 0 - 0.16 m/s and turning speed to −0.80 to 0.80 rad/s. The waypoint is considered reached when the robot is within 0.03 m of it, so the robot does not need to stop at an exact mathematical point. These proportional gains were chosen to provide simple, stable control in the simulation. The 0.16 m/s speed limit keeps the robot from moving too quickly while allowing it to finish the task in a reasonable time. 

### 5.2 LiDAR Based Obstacle Avoidance



## Demo Video

To be added.

## Team Contributions

To be added.

## Generative AI Declaration

To be completed with the actual tools and purposes.

<script>
window.MathJax = {
  tex: {
    inlineMath: [['$', '$'], ['\\(', '\\)']],
    displayMath: [['$$', '$$'], ['\\[', '\\]']]
  }
};
</script>
<script async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-chtml.js"></script>
