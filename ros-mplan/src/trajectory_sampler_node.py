#!/usr/bin/env python

import rospy
from duckietown_mplan import TrajectorySampler


def main():
    rospy.init_node('trajectory_sampler_node', anonymous=False)

    # instantiate standalone trajectory sampler with 10 hz
    trajectory_sampler = TrajectorySampler(standalone=True, frequency=10)


if __name__ == '__main__':
    main()
