[![CircleCI](https://circleci.com/gh/duckietown/duckietown-mplan.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-mplan)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-mplan/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-mplan?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_mplan.svg)](https://pypi.python.org/pypi/duckietown_mplan/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_mplan.svg)](https://pypi.python.org/pypi/duckietown_mplan/)


# Duckietown Mplan

A versatile lane following algorithm for obstacle avoidance


## Installation from source

This is the way to install within a virtual environment created by
using `pipenv`:

    $ pipenv install
    $ pipenv shell
    $ cd lib-mplan
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps


## Unit tests

Run this:

    $ make -C lib-mplan tests-clean tests

The output is generated in the folder in `lib-mplan/out-comptests/`.
