#!/bin/bash

# copy this file in a folder in which you want all the demo stuff for mplan
# use this command to do the setup: ". ./setup_from_blank_folder.bash"

# prerequesits
# - ssh key for github
# - pip
# - virtualenv
# - catkin_build


# virtualenv
virtualenv -p python2.7 venv

source venv/bin/activate

# duckietown world
git clone git@github.com:duckietown/duckietown-world.git

cd duckietown-world

pip install -r requirements.txt

python setup.py develop --no-deps

cd ..

# our package
mkdir -p dt_catkin_ws/src

cd dt_catkin_ws

catkin init

cd src

git clone git@github.com:duckietown/duckietown-mplan2.git

cd duckietown-mplan2

# current fix, as our proper branch is not yet the master
git checkout master

pip install -r lib-mplan/requirements.txt

git submodule update --init --recursive

cd ../..

catkin build

source devel/setup.bash

# launch default demo
roslaunch duckietown_mplan mplan_withviz_demo.launch demo_num:=5
