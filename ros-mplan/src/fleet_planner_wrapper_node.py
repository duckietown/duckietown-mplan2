#!/usr/bin/env python

import math
import rospy
import mplan_msgs.msg as mpmsg
import flock_simulator.msg as fsmsg
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker


#TODO add these variables to rosparam server and read from there
duckie_radius = rospy.get_param('cost_function/obstacles/duckie_bot/radius')
actor_id = "duckie-0"
manual_duckiebot_id = "duckie-1"
sampler_frequency = 0.1
actor_omega = 0
manual_command = 0

fleet_command_pub = rospy.Publisher(
        'flock_simulator/commands', fsmsg.FlockCommand, queue_size=10)
obstacle_pub = rospy.Publisher(
        'obst_avoid/obstacles', mpmsg.Obstacles, queue_size=10)
street_obstruction_pub = rospy.Publisher(
        'obst_avoid/street_obstruction', Marker, queue_size=10)
actor_pub = rospy.Publisher(
        'obst_avoid/actor', mpmsg.Actor, queue_size=10)

def fleetPlannerCb(msg):
    # empty array for saving the obstacles
    obstacle_list = []
    actor_msg = mpmsg.Actor()

    # parse every duckie state to a moving object msg
    # if the duckie is our main controllerd duckie publish it to the actor topic
    # else publish to obstacle topic
    for duck in msg.duckie_states:
        if duck.duckie_id.data == actor_id:
            actor_msg.header = msg.header
            actor_msg.name = duck.duckie_id.data
            actor_msg.moving_object.pose.x = duck.pose.x
            actor_msg.moving_object.pose.y = duck.pose.y
            actor_msg.moving_object.pose.theta = duck.pose.theta
            lx = duck.velocity.linear.x
            ly = duck.velocity.linear.y
            theta = duck.pose.theta
            actor_msg.moving_object.twist.x = math.cos(theta)*lx-math.sin(theta)*ly
            actor_msg.moving_object.twist.y = math.sin(theta)*lx+math.cos(theta)*ly

            actor_msg.moving_object.safety_radius = duckie_radius

            # save omega for command calculation
            actor_omega = duck.pose.theta

        else:
            obstacle = mpmsg.MovingObject()
            obstacle.pose.x = duck.pose.x
            obstacle.pose.y = duck.pose.y
            obstacle.pose.theta = duck.pose.theta
            lx = duck.velocity.linear.x
            ly = duck.velocity.linear.y
            theta = duck.pose.theta
            obstacle.twist.x = math.cos(theta)*lx-math.sin(theta)*ly
            obstacle.twist.y = math.sin(theta)*lx+math.cos(theta)*ly
            obstacle.safety_radius = duckie_radius
            obstacle_list.append(obstacle)


    # create and fill Obstacles msg and publish it
    obstacle_list_msg = mpmsg.Obstacles()
    obstacle_list_msg.header = msg.header
    obstacle_list_msg.moving_objects = obstacle_list
    obstacle_pub.publish(obstacle_list_msg)
    actor_pub.publish(actor_msg)

def commandCb(msg):
    # empty array for saving the duckie commands (just one entry needed)
    command_list = []

    # parse obst_avoid command msg to fleet_planner command msg
    command = fsmsg.DuckieCommand()
    command.duckie_id.data = actor_id
    command.on_rails.data = False
    command.command.linear.x = msg.v
    command.command.linear.y = 0
    command.command.linear.z = 0
    command.command.angular.x = 0
    command.command.angular.y = 0
    command.command.angular.z = msg.omega

    # create and fill command msg and publish it
    commands_msg = fsmsg.FlockCommand()
    commands_msg.header = msg.header
    commands_msg.dt.data = sampler_frequency
    commands_msg.duckie_commands = [command]

    if manual_command != 0:
        manual_duckie_command = fsmsg.DuckieCommand()
        manual_duckie_command.duckie_id.data = manual_duckiebot_id
        manual_duckie_command.on_rails.data = False
        manual_duckie_command.command = manual_command
        commands_msg.duckie_commands.append(manual_duckie_command)

    fleet_command_pub.publish(commands_msg)

def manualCommandCb(msg):
    global manual_command
    manual_command = msg

def streetObstructionCb(msg):
    street_obstruction_pub.publish(msg)

def main():
    rospy.init_node('fleet_planner_wrapper_node', anonymous=False)

    # instantiate standalone worker with at 10 hz
    fleet_planner_sub = rospy.Subscriber(
                '/flock_simulator/state', fsmsg.FlockState, fleetPlannerCb)
    command_sub = rospy.Subscriber(
            '/obst_avoid/twist', mpmsg.Twist2DStamped, commandCb)

    manual_command_sub = rospy.Subscriber(
            '/obst_avoid/manual_duckiebot_twist', Twist, manualCommandCb)
    street_obstruction_sub = rospy.Subscriber(
            'flock_simulator/street_obstruction', Marker, streetObstructionCb )


    rospy.spin()




if __name__ == '__main__':
    main()
