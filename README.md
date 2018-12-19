[![CircleCI](https://circleci.com/gh/duckietown/duckietown-mplan.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-mplan)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-mplan/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-mplan?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_mplan.svg)](https://pypi.python.org/pypi/duckietown_mplan/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_mplan.svg)](https://pypi.python.org/pypi/duckietown_mplan/)


# Duckietown Mplan

A versatile lane following algorithm for obstacle avoidance.

The Obstavoid algorithm is based on a shortest path optimisation problem, which seeks the best way through a weighted, three dimensional space-time grid. The three main pillars necessary for this problem setting are the design of a suitable cost function to define the actor’s behaviour, a graph search algorithm to determine the optimal trajectory and a sampler, which extracts the desired steering commands from a given trajectory and the actor’s position


## Installation from source

This is the way to install within a virtual environment created by
using `pipenv`:

    $ pipenv install
    $ pipenv shell
    $ cd lib-mplan
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps


## Software architecture

![software_architecture](https://user-images.githubusercontent.com/32458198/50254819-d826cf80-03ef-11e9-9b3c-c1209eeb25d0.jpg)


The pipeline is divided up into two main nodes which communicate via topic communication.

[add graph]

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
