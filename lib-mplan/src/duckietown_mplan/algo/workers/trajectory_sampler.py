__all__ = [
    'TrajectorySampler',
]

from worker_base import WorkerBase
from duckietown_mplan.algo.containers import Trajectory
from duckietown_mplan.algo.containers import Obstacle


import rospy
import mplan_msgs.msg as mpmsg
import math
from std_msgs.msg import Empty


class TrajectorySampler(WorkerBase):
    """
    The trajectory sampler. It gets a trajectory and samples it at the current
    time point. From this sample it then calculates the kinematic commands which
    can be passed on to the drive controller.

    Parameters
    ----------
    trajectory: Trajectory
        the most recent trajectory as calculated by the trajectory creator. A
        subscriber will always save the newest published trajectory to this
        member object
    """

    def __init__(self, standalone=True, frequency=-1):
        """
        Call constructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectorySampler, self).__init__(standalone, frequency)
        rospy.loginfo('[TrajectorySampler.__init__] init complete')

    def __del__(self):
        """
        Call destructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectorySampler, self).__del__()

    def init(self):
        """
        Initialise all members of class.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        # initial trajectory, simply holds position
        self.trajectory = Trajectory()
        self.trajectory.start_time = rospy.Time.now()
        self.trajectory.duration = 5
        self.trajectory.ts = 1/self.frequency
        self.trajectory.positions = [[0,0],[0,0]]
        self.trajectory.times = [0,1]
        self.trajectory.updateInterpolation()

        self.actor = Obstacle()

        self.vel_max = rospy.get_param('trajectory_sampler/velocity/max')
        self.omega_max = rospy.get_param('trajectory_sampler/omega/max')
        self.dist_min = rospy.get_param('trajectory_sampler/distance/min')
        self.target_time = rospy.get_param('trajectory_sampler/target_time')
        self.k_vel = rospy.get_param('trajectory_sampler/k_vel')
        self.k_P = rospy.get_param('trajectory_sampler/k_p')
        self.k_I = rospy.get_param('trajectory_sampler/k_i')
        self.k_D = rospy.get_param('trajectory_sampler/k_d')
        self.err = 0
        self.int = 0

    def initIO(self):
        """
        Instantiate all input / output behaviour of worker. This mainly
        includes ros-publishers / -subscribers and advertised services.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        self.trajectory_sub = rospy.Subscriber(
            'obst_avoid/trajectory', mpmsg.TimedPath, self.trajectoryCb)

        self.actor_sub = rospy.Subscriber(
            'obst_avoid/actor', mpmsg.Actor, self.actorCb)

        self.command_pub = rospy.Publisher(
            'obst_avoid/twist', mpmsg.Twist2DStamped, queue_size=10)

        # wait until first trajectory is published from trajectory creator
        rospy.wait_for_message("obst_avoid/trajectory", mpmsg.TimedPath)

    def trajectoryCb(self, data):
        self.trajectory.fromMsg(data)

    def actorCb(self, data):
        self.actor.fromMsg(data.moving_object)

    def advance(self, Ts=1.0):
        """
        Grabs the current obstacle list, stored in self.obstacle_list and uses
        this to populate the cost grid. The cost grid is then passed to the
        trajectory solver which finds an optimal path and stores it as a
        trajectory. This trajectory is then published.

        Parameters
        ----------
        Ts : double, optional
            The time in seconds since the advance method was executed the last time

        Returns
        -------
        none
        """

        # target position on trajectory to be reached after target_time
        x_set, y_set = self.trajectory.getPositionFromTimePoint(rospy.Time.now() + rospy.Duration(self.target_time))

        # position on trajectory, where actor should be now
        x_set_now, y_set_now = self.trajectory.getPositionFromTimePoint(rospy.Time.now())

        # position, angle, velocity of actor
        x_act, y_act, x_act_dot, y_act_dot = self.actor.getState()

        dist_to_target = math.sqrt((x_set - x_act)**2 + (y_set - y_act)**2)

        # velocity needed to reach target positon within target_time
        vel_set = self.k_vel * dist_to_target / self.target_time

        # # angular error and distance to line between trajectory position and target position
        # if(x_set == x_set_now and y_set == y_set_now):
        #     phi_ref = math.atan2(y_set - y_act, x_set - x_act)
        #     phi_est = math.atan2(y_act_dot, x_act_dot)
        #
        #     d_ref = 0
        #     d_est = 0
        #
        # else:
        #     phi_ref = math.atan2(y_set - y_set_now, x_set - x_set_now)
        #     phi_est = math.atan2(y_act_dot, x_act_dot)
        #
        #     d_ref = 0
        #     d_est = math.fabs((y_set - y_set_now) * x_act - (x_set - x_set_now) * y_act + x_set * y_set_now - y_set * x_set_now) / math.sqrt((x_set - x_set_now)**2 + (y_set - y_set_now)**2)
        #
        # ref = (6 * d_ref + 1 * phi_ref)
        # est = (6 * d_est + 1 * phi_est)
        # err = ref - est
        # # print('phi_ref: %f, %f', phi_ref, phi_est)

        d_ref = 0
        d_est = 0

        phi_ref = math.atan2(y_set - y_act, x_set - x_act)
        phi_est = math.atan2(y_act_dot, x_act_dot)

        d_err = d_ref-d_est
        phi_err = phi_ref -phi_est

        if phi_err > math.pi:
            phi_err -= 2*math.pi
        elif phi_err < -math.pi:
            phi_err += 2*math.pi

        err = 0*d_err+1*phi_err

        # PID for omega
        C_P = self.k_P * err
        C_I = self.k_I * (self.int + err)
        C_D = self.k_D * (err - self.err) / Ts

        # store err in memory for derivative and integrate int
        self.err = err
        self.int = self.int + err

        omega_set = C_P + C_I + C_D

        if dist_to_target < self.dist_min:
            vel_set = 0
            omega_set = 0

        #publish
        command_msg = mpmsg.Twist2DStamped()  # TODO, add proper message and populate it
        command_msg.header.stamp = rospy.Time.now()
        command_msg.v = min([vel_set, self.vel_max]) # either calculated value or vel_max is published
        command_msg.omega = min([omega_set, self.omega_max]) # either calculated value or omega_max is published
        self.command_pub.publish(command_msg)

    def shutdown(self):
        """
        Clean up class before process end.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        pass
