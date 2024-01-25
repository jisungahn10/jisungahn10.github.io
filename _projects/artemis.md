---
layout: page
title: ARTEMIS
description: Advanced Robotic Technology for Enhanced Mobility and Improved Stability
img: assets/robocup23/img/artemis.jpg
importance: 1
category: robotics
related_publications: Authors15, ahn2023development,brooks1986robust, brooks1991new,  nakhaeinia2011review, tzafestas2018mobile, vukovic2009new, batinovic2020decentralized, carreras2003proposal
toc: 
    sidebar: left
---

# Context
The Robotics and Mechanisms Laboratory (RoMeLa) participated in RoboCup, an international robot soccer competition with different leagues and robots in order to promote robotics and artificial intelligence research. We entered the Adult-Sized Humanoid (ASH) League where four adult-sized humanoid robots autonomously play a game of soccer against each other (Figure 1). The autonomous robots are required to sense the environment, make decisions, and act based on the decisions to achieve victory. A camera and an inertial measurement unit (IMU) were used for sensing. The sensor information obtained from the camera and the IMU is proccessed to localize the robot, other robots, and the ball in the environment. The processed information is used to make strategic decisions to win games. I developed a strategic game planner based on the subsumption architecture to make timely and informed decisions. 

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/robocup23/img/humanoid_league.png" title="robocup23 humanoid league" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 1: Sweaty (Red) vs RoMeLa (Blue)
</div>

# Subsumption Architecture
I chose the subsumption architecture over other common control architectures such as deliberative control and hybrid control due to several reasons. While the humanoid robots that participate in the ASH League may not be as fast nor agile as human soccer players, the environment is still dynamic enough that decisions need to be made quickly, so no time or computational resources should be spent on maintaining an accurate world model, which is what both the deliberative and hybrid control architectures do. The subsumption architecture is a reactive architecture that uses minimally processed information to make decisions on what to do, reducing the decision-making time. The subsumption architecture is structured in layers as shown in Figure 2, where the system is capable of falling back onto lower systems in the case that higher systems run into problems, allowing the system to be less likely to run into complete system failure. The robustness of the subsumption architecture is very important in the contact sport of soccer where collisions between players happen frequently. Lastly, the subsumption architecture allows for parallel development of subsystems. The ultimate goal of winning the game of soccer is highly complex and is required to be broken down into many subtasks achieved by simpler subsystems. The complete system can be more conveniently developed by incrementally building up from subsystems that achieve simpler tasks, such as walking, to more complex behaviors, such as exploring and scoring.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/robocup23/img/subsumption_architecture.png" title="subsumption architecture" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 2: Subsumption Architecture
</div>

The subsumption architecture used for RoboCup 2023 is shown in Figure 3. The lowest layer is simply walking. The tactical planner uses information from the camera and the IMU to decide where to reposition, subsuming the walking layer. If kicking the ball would result in a more advantageous game state, kicking is activated, inhibiting all the layers below. Finally, if the ball is not found, the player will look for the ball, while repositioning to a more defensive location. The ball is always progressed towards the opposition's goal when the kicking condition and the tactical planner are combined as seen in Figure 4. The purpose of buildup is to move the ball towards the opposition's side, while the purpose of offensive play is to move the ball into the opposition's goal.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/robocup23/img/robocup_subsumption_architecture.png" title="robocup subsumption architecture" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 3: RoboCup 2023 Subsumption Architecture
</div>

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/robocup23/img/desired_ball_progression.png" title="ball progression img" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Figure 4: Buildup and Offensive Plays
</div>

# Reinforcement Learning
I also attempted to develop [strategic game behavior using reinforcement learning](https://jisungahn10.github.io/projects/deeprl/) in another project as well!