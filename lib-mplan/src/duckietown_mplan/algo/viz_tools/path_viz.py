__all__ = [
    'PathViz',
]

import rospy
from nav_msgs.msg import Path
import mplan_msgs.msg as mpmsg
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from duckietown_mplan.algo.workers.worker_base import WorkerBase
from duckietown_mplan.algo.containers import Trajectory


class PathViz(WorkerBase):
    """
    The path vizualizer. It converts the timed path to a nav_msgs Path such that
    rviz can plot it. Also it creates a marker which represents the position on
    the trajectory at which the duckiebot should be at the current moment.

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
        super(PathViz, self).__init__(standalone, frequency)
        rospy.loginfo('[PathViz.__init__] init complete')

    def __del__(self):
        """
        Call destructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(PathViz, self).__del__()

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
        self.trajectory = Trajectory()
        self.trajectory.start_time = rospy.Time.now()
        self.trajectory.duration = 5
        self.trajectory.ts = 1/self.frequency
        self.trajectory.positions = [[0,0],[0,0]]
        self.trajectory.times = [0,1]
        self.trajectory.updateInterpolation()

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

        self.path_pub = rospy.Publisher('obst_avoid_viz/path', Path,
            queue_size=10)

        self.marker_pub = rospy.Publisher(
            'obst_avoid_viz/trajectory/sampled_point', Marker, queue_size=10)

        rospy.wait_for_message("obst_avoid/trajectory", mpmsg.TimedPath)



    def trajectoryCb(self, data):
        # save the trajectory to member variable
        self.trajectory.fromMsg(data)

        # convert the trajectory to a nav_msgs Path and republish
        path = Path()
        path.header.stamp = data.start_time
        path.header.frame_id = 'map'
        for i, elem in enumerate(data.positions):
            pose = PoseStamped()
            pose.header.stamp = data.start_time + rospy.Duration(data.times[i]*data.duration)
            pose.header.frame_id = 'map'
            pose.pose.position.x = data.positions[i].x
            pose.pose.position.y = data.positions[i].y
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0

            path.poses.append(pose)
        self.path_pub.publish(path)

    def advance(self, Ts=1.0):
        """
        Sample the trajectory at the current point and publish a marker which
        represents the standing.

        Parameters
        ----------
        Ts : double, optional
            The time in seconds since the advance method was executed the last time

        Returns
        -------
        none
        """
        x, y = self.trajectory.getPositionFromTimePoint(rospy.Time.now())
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = 'map'
        marker_msg.ns = "obst_avoid_viz"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE;
        marker_msg.action = Marker.ADD;
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 0.1;
        marker_msg.pose.orientation.x = 0
        marker_msg.pose.orientation.y = 0
        marker_msg.pose.orientation.z = 0
        marker_msg.pose.orientation.w = 1
        marker_msg.scale.x = 0.1;
        marker_msg.scale.y = 0.1;
        marker_msg.scale.z = 0.1;
        marker_msg.lifetime = rospy.Duration();
        marker_msg.color.r = 1.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 0.0;
        marker_msg.color.a = 1.0;
        self.marker_pub.publish(marker_msg)

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
