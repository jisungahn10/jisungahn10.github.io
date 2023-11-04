---
layout: page
title: BRAAD
description: Ball Receiving Autonomously Actuated Device
img: assets/braad/img/braad_nolabels.jpg
importance: 5
category: work
related_publications: 
---

# Overview
Robots are often used for tasks that require identifying and safely manipulating objects in an environment. Many robotic manipulators have demonstrated catching objects, which involves tracking the trajectory of a moving target and intercepting it. By grasping the object mid-air with an end-effector, these manipulators successfully demonstrate the impressive speed of their control system. However, the grasping action occurs in an instant, causing sudden, harsh forces on the object and gripper as the object decelerates instantaneously. Thus, without any way to compensate for these forces (such as mechanical compliance in the manipulator joints), this method of interception could damage the object being grasped, especially if it is fragile (such as an egg). In other cases, the manipulator itself could be damaged from the sudden external load, for example, if it was intercepting a heavy package.

BRAAD (Ball Receiving Autonomously Actuated Device) is a robotic manipulator designed to track and gently decelerate a moving target using visual feedback. This will be demonstrated through the task of soccer ball trapping, a maneuver that involves “capturing” a moving soccer ball as it approaches by gently applying a controlled force to decelerate the ball until its velocity reaches zero. In human soccer games, players typically perform trapping by contacting the ball with an outstretched foot and continuously tracking the ball with their foot, keeping it in contact as the ball moves to apply the deceleration force.

BRAAD is designed to actively manipulate the trajectory of a moving soccer ball based on its real-time position and velocity. As the ball approaches, the system will use visual feedback to track the ball’s position and velocity as it approaches in order to determine where the end-effector should be placed to successfully intercept the ball. Before the ball makes contact, the robot will move its end-effector along the ball’s predicted path in order to prevent it from bouncing off. Once contact is established, BRAAD will gently slow down the ball similar to the human case. 

This method of predictive trapping can potentially be extended for use in a variety of robotic manipulators, especially ones that require intercepting moving objects. This will increase the safety of the system for both the object being handled, as well as the robot itself.

# Objective
The objective of the BRAAD robotic system is to actively manipulate the trajectory of a moving target based on that target's real-time position and velocity. The task to represent this objective is modeled as a manipulator robot that traps a rolling ball. The act of trapping occurs most frequently in the sport of soccer, where an athlete cushions an incoming pass and reduces the ball's momentum such that it comes to rest underneath or out in front of the athlete. As the receiving foot moves with the same velocity as the ball, the collision time is extended. The trapping motion of a professional athlete also often directs the ball downward, where momentum can be dissipated into the ground or through deformation of the ball. 

Trapping a ball is different from simply catching a ball, as the act of trapping does not utilize grasping abilities normally associated with manipulator arms. This technique relies purely on the dynamic contact and motion of its end-effector. The ball’s real-time position and velocity can be obtained using vision to plan the trajectory of the end-effector (foot) at impact. 

To this end, BRAAD consists of a combined robotic arm and visual sensor system. The robotic platform is designed to serve as the "leg" with an end-effector "foot" that interacts with balls in the environment, and is comprised of rigid links with dynamixel motors as the joints, as seen in Figure 1. The visual sensor is a camera mounted above the task space, pointing directly downwards to view the ball and robotic arm in the floor plane, as depicted in Figure 2.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/img/BRAAD.jpg" title="braad manipulator" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 1: Manipulator
</div>

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/img/visioncad.jpg" title="overall system" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 2: Overall System
</div>

The system's main objective is to successfully perform a trapping sequence on a rolling ball moving at an arbitrary angle and initial velocity when the ball enters the task space. This involves the end-effector moving to the predicted path of the ball, matching the ball's velocity, establishing contact, and then bringing the ball to a stop.

To successfully perform a trapping maneuver, BRAAD must perform the following sequence of tasks:
1. Sense the position and velocity of the ball as it approaches the workspace
2. Predict the ball's trajectory as it travels through the workspace
3. Identify an optimal point of interception with the ball.
4. Generate joint trajectories

The calculated joint trajectories will then perform the following tasks:
1. Move the end-effector to the interception point
2. Travel along the predicted ball trajectory, and establish contact with the ball at its current velocity (to avoid the ball bouncing off).
3. Slow down the ball by gradually reducing the end-effector velocity until both the robot and ball are at rest.

# Physical Hardware Design
BRAAD's robotic arm is a 3R planar robot comprised of two links and one end-effector. All parts are 3D printed out of ABS. The arm is mounted on a frame made of 80/20 T-slot extrusion that serves as the ground for the arm and outlines the workspace. The arm link lengths were chosen to be 19 cm each, based on the scaling factor compared to the length of an average human leg. Each of the three joints is actuated by a Dynamixel MX-28AR (193:1 gear ratio). The end-effector has a curved surface that decreases the likelihood of the ball bouncing away from the end-effector in the event of a small angle misalignments. This is because the angled surface will cause the ball to bounce towards the center of the curve, which assists in capturing the ball.

All components are wired to interface with a laptop computer. The computer uses MATLAB to coordinate between the vision system, trajectory generation subroutines, and motor controllers.

# Vision System Design
The purpose of this sub-system is to identify the incoming ball, determine its position in operational space, and then continually estimate an incoming ball trajectory that feeds into our controller. A DEPSTECH 4K webcam is mounted 1.5 m above the floor plane on an additional 80/20 frame, giving the camera a view the entire operational space. A bubble leveler was used to ensure the image data would have minimal perspective shifts, allowing one pixel to translate directly to one X-Y coordinate on the floor plane. Distortion is expected to occur on the outer edge of the cameras image, but is assumed to be negligible given the minimal distance between the camera and the workspace. Despite the DEPSTECH's 4K capabilities, an output resolution of 720x1280 pixels was used because high resolution is not a priority and the chosen resolution still clearly displays both the manipulator arm and the blue ball within its field of view. MATLAB's Computer Vision and Image Processing toolboxes was used to conduct a frame by frame analysis to determine the ball's position. The computer vision system does region-based image segmentation based on area and interprets the centroid of the region as the center of the ball with respect to time. Using the 5 most recently recorded position coordinates of the ball and the corresponding time values, a prediction of the ball's trajectory was made based on a first order polynomial. The predicted trajectory and the current position of the end-effector was used to determine the point of interception of the ball (Figure 3).

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/gif/predicted_ball_gif1.gif" title="predicted ball gif1" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/gif/predicted_ball_gif2.gif" title="predicted ball gif2" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 3: Prediction of the Ball's Trajectory
</div>

<!-- The decision to use MATLAB rather than other computer vision applications like OpenCV was grounded in optimizing the development process. Using MATLAB allowed for the simplest and most intuitive integration between project subsystems: vision and motor control. However, this decision brings with it the the drawback of limited processing speed and requires running the computer vision system sequentially with trajectory generation and motor control. -->
<!-- With regards to camera settings, despite the DEPSTECH's 4k capabilities, the team opted to utilize an output resolution of 720x1280 px. High resolution was not a design priority, and the chosen resolution still clearly displays both the maniupulator arm and blue ball within its field of view. -->


# Controller Requirements
To successfully complete the task of trapping, BRAAD's controller must be able to meet the following design requirements:
- Allows for joint speeds sufficient to intercept a ball from an arbitrary position, orientation, and velocity that approaches from outside of the workspace.
- Reaches the interception position and settles within 0.5 seconds. This is to ensure the end-effector is not still moving by the time the ball reaches the manipulable space. The settling time was chosen based on the worst case scenario where the ball is approaching head-on with maximum velocity from the minimum distance detectable by the camera (2 ft or 609 mm).
- Accurately matches the end-effector's velocity to the ball's velocity at the time of contact to prevent the ball from bouncing off.
- Computationally efficient enough to run simultaneously with the vision system.

To meet these requirements, we decided to utilize two controllers in sequence: 
- Stage 1: Joint space decentralized feedforward position control, activated when moving the arm to the interception point.
- Stage 2: Global space decentralized feedforward velocity control, activated to contact the ball at its velocity and decelerate to a rest.

Decentralized control is viable given the high gear reduction ratios of the Dynamixel motors (193:1), but also relies on other assumptions:
- The ball has low initial velocity. This is to meet the "relatively small joint actuation torques" requirement for using decentralized control. We determined a maximum initial velocity of 1 m/s to be sufficient given the downscaled nature of the system. 
- The motors are relatively efficient and links are relatively low mass, so nonlinear behaviors due to coupling can be ignored.
- The ball has negligible mass. This is to simplify the control problem by treating it as motion in free space, ignoring external forces or torques that might be generated from the ball impacting the end-effector during contact.
- The surface is flat. The trajectory of the ball can be modeled as straight line motion.
Decentralized control simplifies the controller significantly due to the linearized system dynamics, which decreases the computational load and increases speed. By skipping the computation required for dynamic controllers, computation time can be drastically reduced. Additionally, during the first stage, where the end-effector's trajectory in global frame is not important, joint space control can be used. This simplifies the process of sending commands to the Dynamixel motors, but requires necessary conversions for the ball which will be moving in X-Y in the global frame.
Feedforward compensation can be utilized since desired trajectories need to be generated ahead of time with the vision system in order to calculate where to move the arm. These trajectories can be fed forward into the controller to help reduce tracking errors that might result from high speeds or accelerations during the relatively quick intercepting motion.

# Controller Design
To trap an incoming ball, the system must perform many preliminary and real-time calculations in order to analyze the ball and generate trajectories to feed into the position or velocity controllers. Since we wanted to reduce computing time whenever possible, certain steps were pre-calculated based on BRAAD's physical parameters to assist the controller in generating desired trajectories. 

## Point of Interception
If the ball's predicted trajectory were to cross the robot's workspace, there are numerous locations where the end-effector could intercept the ball. Manipulability, a measure of how much velocity or force the end-effector could produce, was used as a metric to settle on a single location. The forward kinematics of the 3R system is derived to compute the manipulability. For link lengths $$L_{1}$$, $$L_{2}$$, and $$L_{3}$$, with joint angles $$\theta_{1}$$, $$\theta_{2}$$, and $$\theta_{3}$$, the forward kinematics for the system are:

\begin{equation}
\begin{bmatrix}
x
y
\phi
\end{bmatrix}
=
\begin{bmatrix}
L_{1}c_{1}+L_{2}c_{12}+L_{3}c_{123}
L_{1}s_{1}+L_{2}s_{12}+L_{3}s_{123}
\theta_{1}+\theta_{2}+\theta_{3}
\end{bmatrix}
\end{equation}

These equations were used to find the Jacobian $$J$$ by calculating:

\begin{equation}
J = 
\begin{matrix}
\frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} & \frac{\partial x}{\partial \theta_3} \\
\frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2} & \frac{\partial y}{\partial \theta_3} \\
\frac{\partial \phi}{\partial \theta_1} & \frac{\partial \phi}{\partial \theta_2} & \frac{\partial \phi}{\partial \theta_3}
\end{matrix}
\end{equation}

Performing this calculation yields:

\begin{equation}
J = 
\begin{matrix}
-L_1  s_1-L_2  s_{12}-L_3  s_{123} & L_1 c_1+L_2 c_{12}+L_3 c_{123} & 1 \\
-L_2  s_{12}-L_3  s_{123} & L_2 c_{12}+L_3 c_{123} & 1 \\
-L_3  s_{123} & L_3 c_{123} & 1
\end{matrix}
\end{equation}

From this, we then used the manipulability metric \textit{w} to assess the manipulability for a given point in the workspace. 
The Jacobian was used to assess the manipulability for a given point in the workspace.

\begin{equation}
w = \sqrt{det(JJ^{T})}
\end{equation}

With this metric, the manipulabiliy can be found at several points distributed throughout the workspace to visualize the robot's capabilities. Monte Carlo method is used to analyze the manipulability of the robot across the entire workspace (Figure 4), where the red points indicate regions of higher manipulability. A circular region of highest manipulability can be observed from the visualization of the manipulability.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/img/matlabmanipulability.jpg" title="manipulability analysis" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 4: Monte Carlo Simulation of Manipulability
</div>

Trapping the ball inside the red region is the best case scenario since the end-effector will need to travel linearly along the ball's path while maintaining a constant end-effector orientation. A point with higher manipulability will have a reduced likelihood that the robot will be unable to perform the required maneuvers to cushion the ball.

While this metric is useful for visualization, it is computationally expensive to perform these calculations in real-time, which can slow down the controller. This is undesirable when the system only has fractions of a second to select the best interception point and generate the trajectory to reach it as the ball approaches. Thus, instead of calculating the manipulability metric and evaluating all potential interception points, this process was simplified by designating a "circle of best manipulability", lying in the center of the region of highest manipulability. This circle is taken to have a radius of 0.265 m from the base of the manipulator, and can be seen as red dotted circle in Figure 5.

<div class="col-sm mt-3 mt-md-0">
    {% include figure.html path="assets/braad/img/matlabinterception1.png" title="interception 1" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 5: Circle of Best Manipulability
</div>

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/img/visionpredict.jpg" title="ball prediction" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 6: Ball Trajectory Prediction
</div>

To generate the ball's predicted trajectory, the vision system captures the most recent 5 frames and determines the ball's X-Y position, direction, and speed in the global frame. From these data points, the system calculates a predicted ball trajectory which forms a straight line through the workspace, as seen in Figure 6. If this line intersects the circle of best manipulability, the system selects the intersection that is closest to the end-effector's current position and generates joint trajectories. Otherwise, the system selects the point on the ball's trajectory that is the shortest distance from the circle.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/img/matlabinterception1.png" title="interception 1" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/braad/img/matlabinterception2.png" title="interception 2" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 7: Point of Interception Selection
</div>