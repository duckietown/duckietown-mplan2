__all__ = [
    'CostGridSolver',
]

import rospy
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

import mplan_msgs.msg as mpmsg

from duckietown_mplan.algo.containers import Obstacle
from duckietown_mplan.algo.containers import CostGrid
from duckietown_mplan.algo.containers import Trajectory

class CostGridSolver:
    """Populates the cost grid from the existing objects"""

    def __init__(self):
        pass

    def __del__(self):
        pass

    def solve(self, cost_grid, cost_grid_params):
        """
        Find the optimal path through the cost grid and save the path in a
        trajectory

        Parameters
        ----------
        cost_grid : CostGrid
            a populated cost grid

        Returns
        -------
        Trajectory
            a trajectory corresponding to the optimal path
        """

        assert cost_grid.isPopulated()

        trajectory = Trajectory()

        # takes edge AND node costs into account
        def weight_func(u, v, d):
            node_u_wt = cost_grid.costs.nodes[u].get('node_weight', 0)
            node_v_wt = cost_grid.costs.nodes[v].get('node_weight', 0)
            edge_wt = d.get('weight', 0)
            return node_u_wt/2. + node_v_wt/2. + edge_wt

        # solve the SP problem and print solution for verification
        # print(cost_grid.costs.edges())
        path = nx.dijkstra_path(cost_grid.costs, 'S', 'E', weight_func)

        # convert path object
        path_tf = []
        for waypoint in path[1:len(path)-1]: #to cutoff 'S' and 'E'
            path_tf.append((cost_grid.getXWorld(waypoint[0], waypoint[1], waypoint[2]), cost_grid.getYWorld(waypoint[0], waypoint[1], waypoint[2])))
        # output solution to trajectory object
        # TODO LG check this please
        trajectory.start_time = rospy.Time.now()
        trajectory.duration = cost_grid_params.get('n_t')*cost_grid_params.get('dt')
        trajectory.ts = cost_grid_params.get('dt')
        trajectory.positions = path_tf
        # print(path)
        # print(path_tf)
        trajectory.times = np.linspace(0, 1, len(path_tf))

        return trajectory
