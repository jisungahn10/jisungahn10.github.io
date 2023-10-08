---
layout: page
title: Reinforcement Learning
description: Reinforcement learning approach to strategic game behavior
img: assets/img/3.jpg
importance: 3
category: work
related_publications: einstein1956investigations, einstein1950meaning
giscus_comments: true
---

# Context
The Robotics and Mechanisms Laboratory (RoMeLa) participated in RoboCup, an international robot soccer competition with different leagues and robots in order to promot robotics and artificial intelligence research. We entered the Adult-Sized Humanoid (ASH) League where four adult-sized humanoid robots autonomously play a game of soccer against each other. The autonomous robots are required to sense the environment, make decisions, and act based on the decisions to achieve victory. A camera and an inertial measurement unit (IMU) were used for sensing. The sensor information obtained from the camera and the IMU is proccessed to localize the robot, other robots, and the ball in the environment. The processed information is used to make strategic decisions to win games. I developed a strategic game planner based on the subsumption architecture to make timely and informed decisions. 

# Reinforcement Learning
While it was simple and straightforward to develop basic behaviors and organize them into an augmented finite state machine to make tactical decisions, the behaviors were crafted based on heuristics, which was time consuming, and human estimation, which is not guaranteed to be optimal. There is also no way to evaluate the different behaviors, requiring even more heuristics to decide on which behaviors to keep and which to remove. Reinforcement learning could be used as an approach to quantitatively evaluate the effectiveness of the emergent behavior. Due to the success of reinforcement learning algorithms in board games, such as Chess and Go, and even video games, such as Super Mario and Starcraft, reinforcement learning was explored as a game strategy module. Further success was found in an application of reinforcement learning to a bipedal robot to perform complex behaviors by synthesizing dynamic motions, showing feasibility on real-world. Thus, reinforcement learning is explored as a method to make tactical decisions for a bipedal soccer robot. To do so, it would need to observe the positions of itself, the ball, and the opponents in Cartesian space, resulting in a eight dimensional observation space.

A baseline scenario was defined and developed in a simulation environment to compare reinforcement learning and finite state machine approaches. The environment was built based on the real field dimensions. The scenario starts with the player starting in a random position on the left side of the field and the ball placed in the center of the field. The scenario ends when the ball is kicked into the opposition's goal. The time taken to complete the scenario is averaged over multiple trials to measure the performance of each approach. The standard deviation of the time taken over the multiple trials is used to measure the consistency and reliability of each approach.

## Q-Learning
A basic Q-learning algorithm was first used to train a robot in a grid-based world to kick a ball into the goal. The initial attempts at using Q-learning showed promise, but the algorithm quickly ran into the curse of dimensionality. The algorithm managed to reach ending conditions when the observation space was four dimensional (positions of the robot and the ball), but immediately struggled when the grid was expanded from a 9 by 14 table to 18 by 28 table. In other words, the basic Q-learning algorithm using a grid-based world is not scalable. Furthermore, in order to produce quality decisions, it is necessary for the model to train on an environment that accurately represents the real world, so compromising for a simpler environment with fewer discrete spaces is not an option.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/rl/img/robocup_simulation_environment.png" title="rl simulation environment" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    simulation caption
</div>

## Deep Q Network
To deal with the challenges of dimensionality, Deep Q Network (DQN) was also explored. DQN is a model-free network that does not require its environment's transition functions to make predictions of future states or rewards. Since the planner is correlated to building a planner for the actual soccer game, the RL model-free approach is expected to give a good robust solution. DQN is built on Fitted Q-Iteration, which uses different tricks to stabilize the learning with neural networks. I used stable-baseline3 library, which comes with a set of reliable implementations of Reinforced Learning algorithms. The DQN algorithm in this library has provided us with Vanilla Deep Q learning implementation.

The observation space is continuous, but the action space is discrete. A large amount of time was spent building a custom environment with strategically placed rewards to encourage certain behaviors such as getting closer to the ball and kicking the ball towards the opposition's goal. Negative reward is accrued each time step to avoid making unnecessary repetitive actions. Due to time constraints, the task has been simplified to kicking the ball towards the opposition's goal without any opponents. The opponents were removed from the observation space because designing collisions into the model was causing the agent to freeze in one position.

The final exploration rate influenced how robust and accurate the model was to different types of situations. A low final exploration rate produces a highly constrained action, but is prone to getting stuck in local optimas. A high final exploration rate would require exponentially more epochs, but would be more robust against local optimas. Different models with a final exploration rates of 0.05, 0.3, and 0.5 were trained and the exploration rate with the highest reward was chosen.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/rl/img/DifferentExpDQN.jpg" title="exploration rates" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    simulation caption
</div>

With a set exploration rate, models with different architectures were also trained (Fig. \ref{fig:dqn net arch}). Modifications to the architecture were made because the scenario involves not only finding a path to the ball, but also interacting with it. Thus, more layers were added to represent the additional complexity. More neurons were added to the layers to better represent the states, but no significant performance boost was observed. The architecture that achieved the highest reward was chosen for the algorithm comparison.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/rl/img/DQN_all.jpg" title="dqn architectures" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    simulation caption
</div>


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
