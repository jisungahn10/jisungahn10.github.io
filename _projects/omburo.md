---
layout: page
title: OmBURo
description: Omnidirectional Balancing Unicycle Robot
img: assets/omburo/img/omburo_sim.png
importance: 4
category: work
related_publications: einstein1956investigations, einstein1950meaning
---

# Overview
The Omnidirectional Balancing Unicycle Robot (OmBURo) is a robotic system developed at the Robotics and Mechanisms Laboratory (RoMeLa). OmBURo is a robot that was designed as a mobility mechanism in tight spaces shared with people. In order to operate in the close-spaced environment, OmBURo operates in the close-spaced environment with humans by moving omnidirectionally and dynamically balancing based on the principle of dual-axis wheeled inverted pendulum. OmBURo uses a pair of servo motors to drive one large wheel and a set of rollers along the circumference of the large wheel, allowing for omnidirectional movement. Additionally, by using a single wheel for mobility, OmBURo is capable maneuvering dextrously with a small footprint, which is invaluable in tight-spaces. Without using additional motors or other hardware components to the system, I extended the work on OmBURo by analyzing the dynamic factors that influence the orientation, specifically the yaw, of OmBURo and designed a cascaded multi-loop controller to track a desired yaw rate, using a dynamic phenomenon called gyroscopic precession.

# Gyroscopic Precession

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/omburo/img/coin_convention.png" title="coin convention" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
    </div>
</div>
<div class="caption">
    Figure 1: (Left) Precession, external torque, and angular momentum shown on a coin. (Right) $\theta_{\text{lat}}$ can be seen in a 2-D view.
</div>

Gyroscopic precession is the relationship between the angular momentum of an object and the torque applied to it. I first analyzed this phenomenon on a simple coin (Figure 1) rolling down a surface and extended my analysis to OmBURo (Figure 2). 
