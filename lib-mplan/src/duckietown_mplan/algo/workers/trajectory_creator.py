__all__ = [
    'TrajectoryCreator',
]

import rospy
from std_msgs.msg import Empty
import mplan_msgs.msg as mpmsg
import numpy as np

from worker_base import WorkerBase
from duckietown_mplan.algo.manipulators import CostGridPopulator
from duckietown_mplan.algo.manipulators import CostGridSolver
from duckietown_mplan.algo.manipulators import MapManipulator
from duckietown_mplan.algo.containers import Obstacle
from duckietown_mplan.algo.containers import Trajectory
from visualization_msgs.msg import MarkerArray, Marker

class TrajectoryCreator(WorkerBase):
    """
    The trajectory creator. It gets a list of obstacles and the pose of the
    actor duckiebot. From this it first populates the cost grid and the solves
    it to find the optimal paths. This optimal path is then published as a
    trajectory.

    Parameters
    ----------
    cost_grid_populator: CostGridPopulator
        The worker used to populate a cost grid

    cost_grid_solver: CostGridSolver
        The worker used to obtain a trajectory from a cost grid

    actor: Obstacle
        obstacle object misused for saving the pose, twist and collision-
        information from the actor duckiebot.

    obstacle_list: Obstacle[]
        list of obstacles in the field of view
    """

    def __init__(self, standalone=True, frequency=-1):
        """
        Call constructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectoryCreator, self).__init__(standalone, frequency)
        rospy.loginfo('[TrajectoryCreator.__init__] init complete')

    def __del__(self):
        """
        Call destructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectoryCreator, self).__del__()

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
        # cost grid parameters from ros parameter server
        self.cost_grid_params =	{
            'n_t' : rospy.get_param('cost_grid/depth/time'),
            'n_x' : rospy.get_param('cost_grid/depth/x'),
            'n_y' : rospy.get_param('cost_grid/depth/y'),
            'dt' : rospy.get_param('cost_grid/delta/time'),
            'dx' : rospy.get_param('cost_grid/delta/x'),
            'dy' : rospy.get_param('cost_grid/delta/y')
        }

        self.max_actor_vel = rospy.get_param('velocity/max')

        self.cost_grid_populator = CostGridPopulator(self.cost_grid_params, self.max_actor_vel)
        self.cost_grid_solver = CostGridSolver()
        self.actor = Obstacle()
        self.tile_size = rospy.get_param('duckietown/tile_size')

        # TODO add here namespace for multiple actors
        self.actor.x = rospy.get_param('actor-0/x_pos_set')*self.tile_size
        self.actor.y = rospy.get_param('actor-0/y_pos_set')*self.tile_size

        self.num_obstructions = rospy.get_param('num_obst') + rospy.get_param('num_obst_rand')
        self.obstacle_list = []
        self.street_obstructions = []
        self.map_manipulator = MapManipulator(self.tile_size)

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
        self.actor_sub = rospy.Subscriber(
            'obst_avoid/actor', mpmsg.Actor, self.actorCb)

        self.obstacle_sub = rospy.Subscriber(
            'obst_avoid/obstacles', mpmsg.Obstacles, self.obstacleCb)

        self.street_obstruction_sub = rospy.Subscriber(
            'flock_simulator/street_obstructions', MarkerArray, self.streetObstructionCb)

        self.trajectory_pub = rospy.Publisher(
            'obst_avoid/trajectory', mpmsg.TimedPath, queue_size=10)

        self.cost_grid_viz_pub = rospy.Publisher(
            'obst_avoid/cost_grid', MarkerArray, queue_size=10)

        self.tile_pub = rospy.Publisher(
            'obst_avoid/tiles', MarkerArray, queue_size=10)


        map = rospy.wait_for_message("duckietown_map", MarkerArray)
        self.tiles = self.map_manipulator.parseMapToTileList(map)
        self.tile_current = self.map_manipulator.findClosestTile(self.actor.x*self.tile_size, self.actor.y*self.tile_size, self.tiles)
        self.tile_next = self.map_manipulator.getNextTile(self.tile_current, self.tiles)

    def actorCb(self, data):
        self.actor.fromMsg(data.moving_object)

    def obstacleCb(self, data):
        self.obstacle_list = []
        for obstacle_msg in list(data.moving_objects):
            new_obstacle = Obstacle()
            new_obstacle.fromMsg(obstacle_msg)

            self.obstacle_list.append(new_obstacle)

    def streetObstructionCb(self, data):
        self.street_obstructions = []

        for i in range(0, self.num_obstructions):
            street_obstruction = Obstacle()
            street_obstruction.fromMarkerMsg(data.markers[i])
            self.street_obstructions.append(street_obstruction)

    def publishTiles(self, tile1, tile2):
        marker_msg = MarkerArray()
        marker_msg.markers.append(self.map_manipulator.getMarker(1, tile1))
        marker_msg.markers.append(self.map_manipulator.getMarker(2, tile2))
        self.tile_pub.publish(marker_msg)

    def advance(self, Ts=1.0):
        """
        Grabs the trajectory object, stored in self.trajectory and samples it.
        Then publishes the calculated command.

        Parameters
        ----------
        Ts : double, optional
            The time in seconds since the advance method was executed the last time

        Returns
        -------
        none
        """

        if self.map_manipulator.distToTile(self.actor.x, self.actor.y, self.tile_current) > self.map_manipulator.distToTile(self.actor.x, self.actor.y, self.tile_next):
            self.tile_current = self.tile_next
            self.tile_next = self.map_manipulator.getNextTile(self.tile_current, self.tiles)

        cost_grid_origin = self.map_manipulator.getCostGridOrigin(self.tile_current, self.tile_next, self.actor)

        dist_to_centerline = self.map_manipulator.getDistToCenterline(self.tile_current, self.actor.x, self.actor.y)

        # get the filled cost grid
        cost_grid = self.cost_grid_populator.populate(self.actor, self.obstacle_list, self.street_obstructions, self.cost_grid_params, self.max_actor_vel, cost_grid_origin, dist_to_centerline)

        # solve the cost grid for a trajectory
        trajectory = self.cost_grid_solver.solve(cost_grid, self.cost_grid_params)

        # convert the trajectory to a msg
        trajectory_msg = trajectory.toMsg()

        # publish trajectory msg
        self.trajectory_pub.publish(trajectory_msg)

        # publish cost_grid msg
        cost_grid_marker = cost_grid.toVizMsg(self.cost_grid_params)
        self.cost_grid_viz_pub.publish(cost_grid_marker)

        self.publishTiles(self.tile_current, self.tile_next)

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
