[![CircleCI](https://circleci.com/gh/duckietown/duckietown-mplan.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-mplan)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-mplan/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-mplan?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_mplan.svg)](https://pypi.python.org/pypi/duckietown_mplan/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_mplan.svg)](https://pypi.python.org/pypi/duckietown_mplan/)


# Duckietown Mplan

A versatile lane following algorithm for obstacle avoidance.

The Obstavoid algorithm is based on a shortest path optimisation problem, which seeks the best way through a weighted, three dimensional space-time grid. The three main pillars necessary for this problem setting are the design of a suitable cost function to define the actor’s behaviour, a graph search algorithm to determine the optimal trajectory and a sampler, which extracts the desired steering commands from a given trajectory and the actor’s position.

![demo_1_no_cost_grid](https://user-images.githubusercontent.com/32458198/50254958-56837180-03f0-11e9-8e8b-1a34f42c1285.gif)

## Installation from source


### Step 1 - 2 with no effort

If you are *lazy* you can just create a folder on your computer in which we will install everything you need for the demo and which you can easily delete afterwards. Just copy this file [here](https://raw.githubusercontent.com/duckietown/duckietown-mplan2/master/setup/setup_from_blank_folder.bash) in to the folder an run it in your folder with:

```
$ . ./setup_from_blank_folder.bash
```


### Step 1: Virtual environment setup

Make sure you have the prerequisites installed. We recommend running the whole setup in a virtual environment using `python2.7`.

create a virtual environment with
```
$ virtualenv -p python2.7 venv
```
activate the virtual environment
```
$ source venv/bin/activate
```

Install duckietown-world now in the virtual environment, as we depend on libraries from there - for instructions see [here](https://github.com/duckietown/duckietown-world)


### Step 2: Getting the mplan code

Clone the mplan-repo with the following command. Make sure you are inside the `src` folder of a catkin workspace. If you do not have a catkin workspace set up follow these [instructions](https://github.com/duckietown/duckietown-mplan/wiki/Setting-up-a-catkin-workspace).
```
$ git clone git@github.com:duckietown/duckietown-mplan2.git
```

Enter the repo
```
$ cd duckietown-mplan2
```

Install the additional requirements using
```
$ pip install -r lib-mplan/requirements.txt
```

Load the submodules and build the workspace
```
$ git submodule update --init --recursive
```

Build the workspace from the initial folder
```
$ cd ../..
$ catkin build
```
Run `catkin_make` instead if you don't use `python-catkin-tools`.

Next, source your workspace using
```
$ source devel/setup.bash
```

### Step 3: Running the demo

Run the demo including a visualization in rviz with 
```
$ roslaunch duckietown_mplan mplan_withviz_demo.launch demo_num:=5

```
With the parameter `demo_num` you can select a specific scenario. The scenarios are as follows:
 * 1 : dynamic passing of a moving object
 * 2 : passing of a static object with a dynamic object on the other side of the street
 * 3 : blocked road
 * 4 : street passing duckie
 * 5 : multiple obstacles and curves
  
### Step 4: Teleoperating another duckiebot (optional)


In the next step we will take control of another duckiebot, to see how the actor and the obstavoid algorithm will react to it.

Keep the simulation from before running and in a new terminal launch (don't forget to activate your virtual environment and source the catkin workspace)
```
$ rosrun duckietown_mplan duckiebot_teleop_node.py
```
Using 'i', 'j', 'l', ',' you can now teleoperate another duckiebot. With 'q', 'w' you can in-/ decrease it's speed. Make sure to keep the terminal selected, else the keyboard inputs will not be processed.


## Troubleshooting 

 * 1 : Networkx library was not found: Double check that all installations were completed IN the virtual environment, especially the requirements of duckietown-world.
 * 2 : duckietown-world was not found: Double check that all installations were completed IN the virtual environment, especially the setup of duckietown-world.
 * 3 : The duckiebot does not see and crashes in obstacles. Have you spawned to many obstacles / actors? We have seen that too many obstacles / actors introduce a delay into the simulation which makes it impossible to avoid them in an effective manner.
 


## Software architecture

![software_architecture](https://user-images.githubusercontent.com/32458198/50254819-d826cf80-03ef-11e9-9b3c-c1209eeb25d0.jpg)


The pipeline is divided up into two main nodes which communicate via topic communication.

**/trajectory_creator_node [frequency: 10hz]**
* **Input**: */flock_simulator/ state* and */flock_simulator/street_obstruction*: These topics contain position, velocity and size infromation of all obstacles as well as information about the street and where the actor is at the moment. 
* **Computation**: 
**cost_grid_populator:** This manipulator uses the information stored in the obstacles and evaulates the cost function at the discretized points of the cost_grid.
**cost_grid_solver:** This manipulator finds an optimal path in the cost_grid while minimizing total cost.
* **Output:** */obst_avoid/trajectory*: This topic contains a target trajectory for the current cost_grid.

**/trajectory_sampler_node [frequency: 10 hz]**
* **Input**: */obst_avoid/trajectory*: This topic contains a target trajectory for the current cost_grid.
* **Computation**: 
**trajectory_sampler:** This manipulator uses the trajectory and derives the steering commands for the actor duckiebot.
* **Output:** */obst_avoid/trajectory*: This topic contains the current target linear and angular velocity of the bot.


## Unit tests - TODO

Run this:

    $ make -C lib-mplan tests-clean tests

The output is generated in the folder in `lib-mplan/out-comptests/`.


