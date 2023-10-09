---
layout: page
title: LIMMS
description: Latching Intelligent Modular Mobility System
img: assets/limms/img/limms_background.png
importance: 2
category: work
related_publications: zhu2022feasibility, fernandez2022self, fernandez2024self
---

# Overview
Latching Intelligent Modular Mobile System (LIMMS) is a 6 degree of freedom symmetric serial robot where each end can detach and reattach to function as the base or the end-effector. LIMMS was designed as a robotic solution to package transportation by minimizing spatial footprint while maintaining task scalability. Check out the paper by the Robotics and Mechanisms Laboratory (RoMeLa) for more information and details on the feasibility of LIMMS and its potential as a transportation solution.

# Inverted Pendulum Mode
<div class="col-sm mt-3 mt-md-0">
    {% include figure.html path="assets/limms/img/inverted_pendulum.png" title="inverted pendulum image" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 1: Inverted Pendulum
</div>
In order for LIMMS to operate as the package’s mobility system, LIMMS needs to be able to move individually to proper locations in the right orientation around the package. To achieve individual mobility for LIMMS, I modeled LIMMS as an inverted pendulum (Figure 1), which is already well established in literature and learning curriculums and even in commercial products such as Segways (Figure 2).

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/img/segway.jpg" title="segway image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="row">
    <div class="col-sm mt-3 mt-md-0">

    </div>
    <div class="col-sm mt-3 mt-md-0">
        <div class="caption">
            Figure 2: Ninebot by Segway E+
        </div>
    </div>
</div>

I imported the URDF of the LIMMS module into PyBullet to verify the inverted pendulum model as a mobility mode as seen in Figure 3. 

<div class="col-sm mt-3 mt-md-0">
    {% include figure.html path="assets/limms/gifs/limms_mip_sim.gif" title="segway simulation gif" class="img-fluid rounded z-depth-1" %}
</div>
<div class="caption">
    Figure 3: LIMMS Simulation
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
<div class="row">
    <div class="caption">
        Position Tracking
    </div>
    <div class="caption">
        Driving Forwards
    </div>
    <div class="caption">
        Turning 
    </div>
</div>

I also implemented the controller on hardware. I placed the inertial measurement unit (IMU) on the center of LIMMS and communicated with the IMU through WiFi. Unfortunately, the communication with the IMU was unstable and had high latency (Figure 4). The instability of the connection can be observed by the late actuation of the motors during changes in the pitch of LIMMS. The lack of latency can be observed in simulation by how the wheels can never reach the location below the center of mass of LIMMS in time, even with a high I gain in the balancing PID controller. I verified the effect of latency on balancing by running the simulation in both 90 Hz and 240 Hz (Figures 5 and 6). The simulation and hardware implemenations with low latency show similar behavior of how the wheels are increasingly late.

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
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        <div class="caption">
            Figure 4: LIMMS Hardware on Low Frequency and Stability Communication
        </div>
    </div>
    <div class="col-sm mt-3 mt-md-0">
        <div class="caption">
            Figure 5: LIMMS Simulation (90Hz)
        </div>
    </div>
    <div class="col-sm mt-3 mt-md-0">
        <div class="caption">
            Figure 6: LIMMS Simulation (240Hz)
        </div>
    </div>
</div>

To address the issue of latency and connection instability, the IMU was switched to an IMU from MicroStrain and the connection was switched from WiFi to serial. After making the modifications, the hardware was able to successfully balance. Additionally, because the hardware was not run on battery, but instead was run through a wire, the dynamics are different from simulation, which means the dynamic equilibrium is different on hardware. Through trial and error, I found that the difference in the equilibrium angle was about 9 degrees. Below are videos of the balancing system.

<iframe src="https://drive.google.com/file/d/1SvMeZ8K4MBNF0pZ8YqCoMMwzsbhvQkkX/preview" width="640" height="480" allow="autoplay"></iframe>

<iframe src="https://drive.google.com/file/d/1M7NRzYDS4kqp-1rv-AXdMkJleVUMpdr8/preview" width="640" height="480" allow="autoplay"></iframe>

<iframe src="https://drive.google.com/file/d/13a1fIrWekL8VJFyWG-283-rx1ZzFTUtJ/preview" width="640" height="480" allow="autoplay"></iframe>


# Stable Orientation using Optimization for Tri-wheeled Mode
While the inverted pendulum method of mobility may be interesting, it is not necessarily feasible. The stability of the module may be influenced in the roll direction if the surface is not flat, which is not guaranteed in human environments. From an energy efficiency perspective, the module is required to continuously be operational and consume energy to stay balancing. At RoMeLa, we came up with a new mode called Tri-wheeled mode where two LIMMS modules are connected to each other and form a tricycle configuration. However, as seen in the video, the Tri-wheeled mode occupies far more space compared to the inverted pendulum mode. 

<iframe src="https://drive.google.com/file/d/1Wt6Fxywwo53e-WAJfcoaqbFrwoyWlKTA/preview" width="640" height="480" allow="autoplay"></iframe>

I used SciPy's optimization library to find the optimal configuration of the Tri-wheeled mode. I constrained the lateral and longitudinal widths of the mode and maximized the area of the support polygon. Below are videos and figures that show the effet of different constraints. When the depth of the wheels are not constrained, it can be observed that the wheels stretch out as far as possible to maximize the support polygon area (Figure 7). The configuration seems to "bunch up" when both the width and depth are constrained (Figures 8 and 9).

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        <iframe src="https://drive.google.com/file/d/1aAeb_eVVQErjWPOLkcW6FyKf0ocEAr8m/preview" width="640" height="480" allow="autoplay"></iframe>
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/img/widthC_depthU.png" title="limms triw flat" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 7: Constraints only on Width
</div>

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        <iframe src="https://drive.google.com/file/d/1fA2WtSVcnpO85GA4_lbUmaWLuERl0E74/preview" width="640" height="480" allow="autoplay"></iframe>
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/img/widthvaryingC_depthconstantC.png" title="limms triw easier" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 8: Constraints on Width and Depth
</div>

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        <iframe src="https://drive.google.com/file/d/1dJpD5Yc7Njt8vV3OwXwnai88lH9fxilb/preview" width="640" height="480" allow="autoplay"></iframe>
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/limms/img/narrower_widthvaryingC_depthconstantC.png" title="limms triw harder" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 9: Increased Constraints on Width and Depth
</div>
