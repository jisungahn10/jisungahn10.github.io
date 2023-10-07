---
layout: page
title: Latching Intelligent Modular Mobility System (LIMMS)
description: a project with a background image
img: assets/img/12.jpg
importance: 1
category: work
related_publications: einstein1956investigations, einstein1950meaning
---

# Overview
Latching Intelligent Modular Mobile System (LIMMS) is a 6 degree of freedom symmetric serial robot where each end can detach and reattach to function as the base or the end-effector. LIMMS was designed as a robotic solution to package transportation by minimizing spatial footprint while maintaining task scalability. Check out the paper by the Robotics and Mechanisms Laboratory (RoMeLa) for more information and details on the feasibility of LIMMS and its potential as a transportation solution.

# Inverted Pendulum Mode
In order for LIMMS to operate as the package’s mobility system, LIMMS needs to be able to move individually to proper locations in the right orientation around the package. To achieve individual mobility for LIMMS, I modeled LIMMS as an inverted pendulum, which is already well established in literature and learning curriculums and even in commercial products such as Segways.

I imported the URDF of the LIMMS module into PyBullet to verify the inverted pendulum model as a mobility mode. I implemented two PID controllers by actuating the wheel motors, holding all the other joint motors in place, and using the pitch of the center piece at the top of LIMMS as feedback to regulate the orientation of LIMMS. Through trial and error, I found the balancing pitch where LIMMS would mostly track a single position. One PID loop was used to regulate the orientation of LIMMS, while the other was used to track a certain desired position in the environment. Examples of LIMMS tracking a position, driving forwards, and turning are shown below.

I also implemented the controller on hardware. I placed the inertial measurement unit (IMU) on the center of LIMMS and communicated with the IMU through WiFi. Unfortunately, the communication with the IMU was unstable and had high latency. The instability of the connection can be observed by the late actuation of the motors during changes in the pitch of LIMMS. The lack of latency can be observed in simulation by how the wheels can never reach the location below the center of mass of LIMMS in time, even with a high I gain in the balancing PID controller. I verified the effect of latency on balancing by running the simulation in both 90 Hz and 240 Hz. The simulation and hardware videos of low latency show similar behavior of how the wheels are increasingly late



Every project has a beautiful feature showcase page.
It's easy to include images in a flexible 3-column grid format.
Make your photos 1/3, 2/3, or full width.

To give your project a background in the portfolio page, just add the img tag to the front matter like so:

    ---
    layout: page
    title: project
    description: a project with a background image
    img: /assets/img/12.jpg
    ---

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/1.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/3.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/5.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Caption photos easily. On the left, a road goes through a tunnel. Middle, leaves artistically fall in a hipster photoshoot. Right, in another hipster photoshoot, a lumberjack grasps a handful of pine needles.
</div>
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/5.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    This image can also have a caption. It's like magic.
</div>

You can also put regular text between your rows of images.
Say you wanted to write a little bit about your project before you posted the rest of the images.
You describe how you toiled, sweated, *bled* for your project, and then... you reveal its glory in the next row of images.


<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/6.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.html path="assets/img/11.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    You can also have artistically styled 2/3 + 1/3 images, like these.
</div>


The code is simple.
Just wrap your images with `<div class="col-sm">` and place them inside `<div class="row">` (read more about the <a href="https://getbootstrap.com/docs/4.4/layout/grid/">Bootstrap Grid</a> system).
To make images responsive, add `img-fluid` class to each; for rounded corners and shadows use `rounded` and `z-depth-1` classes.
Here's the code for the last row of images above:

{% raw %}
```html
<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/6.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.html path="assets/img/11.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
```
{% endraw %}
