---
layout: page
title: Behavior Module Development using Finite State Machine and Reinforcement Learning
description: a project with a background image
img: assets/limms/img/limms_background.png
importance: 1
category: work
related_publications: einstein1956investigations, einstein1950meaning
---

# Context
The Robotics and Mechanisms Laboratory (RoMeLa) participated in an international humanoid robot soccer competition called RoboCup 2023. 
Latching Intelligent Modular Mobile System (LIMMS) is a 6 degree of freedom symmetric serial robot where each end can detach and reattach to function as the base or the end-effector. LIMMS was designed as a robotic solution to package transportation by minimizing spatial footprint while maintaining task scalability. Check out the paper by the Robotics and Mechanisms Laboratory (RoMeLa) for more information and details on the feasibility of LIMMS and its potential as a transportation solution.

<iframe src="https://drive.google.com/file/d/1M8qanaKonpSOMZVB9Go4K7YUTFzE28Lt/preview" width="640" height="480" allow="autoplay"></iframe>

# Inverted Pendulum Mode
In order for LIMMS to operate as the package’s mobility system, LIMMS needs to be able to move individually to proper locations in the right orientation around the package. To achieve individual mobility for LIMMS, I modeled LIMMS as an inverted pendulum, which is already well established in literature and learning curriculums and even in commercial products such as Segways.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/img/segway.jpg" title="segway image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/img/inverted_pendulum.png" title="inverted pendulum image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Segway and Inverted Pendulum
</div>

I imported the URDF of the LIMMS module into PyBullet to verify the inverted pendulum model as a mobility mode. 

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/gifs/limms_mip_sim.gif" title="segway simulation gif" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    simulation caption
</div>

I implemented two PID controllers by actuating the wheel motors, holding all the other joint motors in place, and using the pitch of the center piece at the top of LIMMS as feedback to regulate the orientation of LIMMS. Through trial and error, I found the balancing pitch where LIMMS would mostly track a single position. One PID loop was used to regulate the orientation of LIMMS, while the other was used to track a certain desired position in the environment. Examples of LIMMS tracking a position, driving forwards, and turning are shown below.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/gifs/limms_position_tracking_sim.gif" title="position tracking gif" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/gifs/limms_driving_forwards_sim.gif" title="driving forwards gif" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/gifs/limms_turning_sim.gif" title="turning gif" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    simulation caption
</div>

I also implemented the controller on hardware. I placed the inertial measurement unit (IMU) on the center of LIMMS and communicated with the IMU through WiFi. Unfortunately, the communication with the IMU was unstable and had high latency. The instability of the connection can be observed by the late actuation of the motors during changes in the pitch of LIMMS. The lack of latency can be observed in simulation by how the wheels can never reach the location below the center of mass of LIMMS in time, even with a high I gain in the balancing PID controller. I verified the effect of latency on balancing by running the simulation in both 90 Hz and 240 Hz. The simulation and hardware videos of low latency show similar behavior of how the wheels are increasingly late.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/gifs/limms_hardware_lowfreq.gif" title="limms hardware wifi gif" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/gifs/limms_90hz_sim.gif" title="limms 90hz sim gif" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/gifs/limms_240hz_sim.gif" title="limms 240hz sim gif" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    simulation caption
</div>

To address the issue of latency and connection instability, the IMU was switched to an IMU from MicroStrain and the connection was switched from WiFi to serial. After making the modifications, the hardware was able to successfully balance. Additionally, because the hardware was not run on battery, but instead was run through a wire, the dynamics are different from simulation, which means the dynamic equilibrium is different on hardware. Through trial and error, I found that the difference in the equilibrium angle was about 9 degrees. Below are videos of the balancing system.
