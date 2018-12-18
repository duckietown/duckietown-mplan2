__all__ = [
    'Trajectory',
]


import rospy
import numpy as np
from std_msgs.msg import Empty
from mplan_msgs.msg import TimedPath
from mplan_msgs.msg import Vector2D
from scipy.interpolate import interp1d

class Trajectory:
    """
    Container object for representation of a generic (timed) 2d trajectory.

    The trajectory is defined to start at time self.start_time and be of duration
    self.duration. The theoretical setpoint values are stored in the
    self.positions array. The normalized time corresponding to each trajectory
    point is saved in the self.times array. The bounds for the trajectory values
    are saved in the self.lower_bounds / self.upper_bounds arrays.

    For more information on the member variables see their specific
    documentation.

    Parameters
    ----------
    start_time: rospy.Time
        time object represanting the start time of trajectory

    duration: float
        exact duration which the whole trajectory should take to execute. Has to
        be greater than 0.

    ts: float
        The timestep duration between consecutive points of trajectory.
        duration / ts = n.

    positions: numpy.ndarray<float>
        n x 2 numpy array containing the setpoint positions of the trajectory.
        First column corresponds to x values, second column to y values.

    times: numpy.ndarray<float>
        n x 1 numpy array containing the temporal setpoint of each trajectory
        point. Values are scaled to unit duration, meaning the first times entry
        will always be 0, the last 1.

    lower_bounds: numpy.ndarray<float>
        n x 2 numpy array containing the positional lower bounds of the
        trajectory. First column corresponds to x values, second column to y
        values.

    upper: numpy.ndarray<float>
        n x 2 numpy array containing the positional upper bounds of the
        trajectory. First column corresponds to x values, second column to y
        values.
    """

    def __init__(self):
        self.start_time = rospy.Time.now()
        self.duration = 0
        self.ts = 0
        self.positions = []
        self.times = []
        self.lower_bounds = []
        self.upper_bounds = []

    def __del__(self):
        pass

    def __str__(self):
        return self.positions.__str__()

    def updateInterpolation(self):
        x = [elem[0] for elem in self.positions]
        y = [elem[1] for elem in self.positions]
        self.f_x = interp1d(self.times, x, kind='linear')
        self.f_y = interp1d(self.times, y, kind='linear')


    def getPositionFromTimePoint(self, time_point):
        """
        get the positional set point corresponding to the unnormalized time t.

        Parameters
        ----------
        time_point: rospy.Time
            the time point for which the trajectory set point shall be returned

        Returns
        -------
        numpy.ndarray<float>
            1 x 2 numpy array containing the x and y positional set point
            correspoding to the given time.
        """
        # convert the time_point to a duration
        time_duration = (time_point - self.start_time).to_sec()

        # call the main getPosition method with the duration argument
        return self.getPositionFromDuration(time_duration)

    def getPositionFromDuration(self, time_duration):
        """
        get the positional set point corresponding to the unnormalized time t.

        Parameters
        ----------
        time_duration: float
            the unnormalized duration in seconds since the start of the
            trajectory

        Returns
        -------
        numpy.ndarray<float>
            1 x 2 numpy array containing the x and y positional set point
            correspoding to the given time.
        """
        # assert we are not looking out of trajectory range
        # assert time_duration < self.duration
        if time_duration > self.duration:
            rospy.logwarn('[Trajectory.getPositionFromDuration] tried to acces point out of bounds, returning last point on path')
            return self.f_x(1), self.f_y(1)

        # scale time to unit duration
        normalized_time = time_duration/self.duration

        # find index of self.times corespoding to time_duration
        idx = self.find_nearest(self.times, time_duration/self.duration)

        # return respective points
        return self.f_x(normalized_time), self.f_y(normalized_time)

    def find_nearest(self, array, value):
        """
        find the index of the element being the closest to value in array

        Parameters
        ----------
        array: tupel
            an array containing the values
        value: float
            the value searched for

        Returns
        -------
        int
            index of closest element in array
        """
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return idx

    def toMsg(self):
        """
        convert the instance to a ros message which can be published

        Parameters
        ----------
        none

        Returns
        -------
        mplan_msgs.msg.TimedPath
            the message containing all information from the instance of this
            class
        """
        pos_list = []
        for elem in self.positions:
            vec = Vector2D()
            vec.x = elem[0]
            vec.y = elem[1]
            pos_list.append(vec)

        msg = TimedPath()
        msg.start_time = self.start_time
        msg.duration = self.duration
        msg.ts = self.ts
        msg.positions = pos_list
        msg.times = self.times
        msg.lower_bounds = self.lower_bounds
        msg.upper_bounds = self.upper_bounds
        return msg

    def fromMsg(self, msg):
        """
        so to say a copy constructor from a msg

        Parameters
        ----------
        msg: mplan_msgs.msg.TimedPath

        Returns
        -------
        none
        """
        pos_list = []
        for elem in msg.positions:
            pos_list.append([elem.x, elem.y])

        self.start_time = msg.start_time
        self.duration = msg.duration
        self.ts = msg.ts
        self.positions = pos_list
        self.times = msg.times
        self.lower_bounds = msg.lower_bounds
        self.upper_bounds = msg.upper_bounds

        self.updateInterpolation()
