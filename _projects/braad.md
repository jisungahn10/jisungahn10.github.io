---
layout: page
title: BRAAD
description: Ball Receiving Autonomously Actuated Device
img: 
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

To this end, BRAAD consists of a combined robotic arm and visual sensor system. The robotic platform is designed to serve as the "leg" with an end-effector "foot" that interacts with balls in the environment, and is comprised of rigid links with dynamixel motors as the joints, as seen in Fig. \ref{fig:braad}. The visual sensor is a camera mounted above the task space, pointing directly downwards to view the ball and robotic arm in the floor plane, as depicted in Fig. \ref{fig:visioncad}. 

The system's main objective is to successfully perform a trapping sequence on a rolling ball moving at an arbitrary angle and initial velocity when the ball enters the task space. This involves the end-effector moving to the predicted path of the ball, matching the ball's velocity, establishing contact, and then bringing the ball to a stop.

To successfully perform a trapping maneuver, BRAAD must perform the following sequence of tasks:
1. Sense the position and velocity of the ball as it approaches the workspace
2. Predict the ball's trajectory as it travels through the workspace
3. Identify an optimal point of interception with the ball.
4. Generate joint trajectories

The calculated joint trajectories will then perform the following tasks:
1. Move the end-effector to the interception point
2. Travel along the predicted ball trajectory, and establish contact with the ball at its current velocity (to avoid the ball bouncing off).
3. Slow down the ball by bradually reducing the end-effector velocity until both the robot and ball are at rest.

