__all__ = [
    'CostGrid',
]

import numpy as np
import networkx as nx
from visualization_msgs.msg import MarkerArray, Marker


class CostGrid:
    """

    Parameters
    ----------
    populated: bool
        represents wheter the cost grid has been populated. Has to be set
        manually by the populator object after population

    costs: nx.DiGraph
        the graph representing the cost grid
    """

    def __init__(self):
        """
        Create cost grid with start and end nodes

        Parameters
        ----------
        empty

        Returns
        -------
        empty
        """
        self.populated = False

        # initialize Graph object
        self.costs = nx.DiGraph()

        # initialize theoretical start and end node (data values unused)
        self.costs.add_node('S', x_pos=0.0, x_world=0.0, y_pos=0.0, y_world=0.0, t_pos=0.0, node_weight=0.0)
        self.costs.add_node('E', x_pos=0.0, x_world=0.0, y_pos=0.0, y_world=0.0, t_pos=0.0, node_weight=0.0)

    def __del__(self):
        pass

    def isPopulated(self):
        return self.populated

    def setCost(self, x, y, t, cost):
        """
        Get the value of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point
        cost: float
            cost value of the desired cost point

        Returns
        -------
        empty
        """
        self.costs.nodes[(x, y, t)]['node_weight'] = cost

    def getCost(self, x, y, t):
        """
        Get the value of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            value of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['node_weight']

    def setXPos(self, x, y, t, x_pos):
        """
        Set the x_pos of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point
        x_pos: float
            x_pos value of the desired cost point

        Returns
        -------
        empty
        """
        self.costs.nodes[(x, y, t)]['x_pos'] = x_pos

    def getXPos(self, x, y, t):
        """
        Get the x_pos of the cost grid at a certain node in the cost grid frame

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            x_pos of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['x_pos']

    def setXWorld(self, x, y, t, x_world):
        """
        Set the x_world of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point
        x_world: float
            x_world value of the desired cost point

        Returns
        -------
        empty
        """
        self.costs.nodes[(x, y, t)]['x_world'] = x_world

    def getXWorld(self, x, y, t):
        """
        Get the x_world of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            x_world of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['x_world']

    def setYPos(self, x, y, t, y_pos):
        """
        Set the y_pos of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point
        y_pos: float
            y_pos value of the desired cost point

        Returns
        -------
        empty
        """
        self.costs.nodes[(x, y, t)]['y_pos'] = y_pos

    def getYPos(self, x, y, t):
        """
        Get the y_pos of the cost grid at a certain node in the cost grid frame

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            y_pos of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['y_pos']

    def setYWorld(self, x, y, t, y_world):
        """
        Set the y_world of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point
        y_world: float
            y_world value of the desired cost point

        Returns
        -------
        empty
        """
        self.costs.nodes[(x, y, t)]['y_world'] = y_world

    def getYWorld(self, x, y, t):
        """
        Get the y_world of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            y_world of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['y_world']

    def setTPos(self, x, y, t, t_pos):
        """
        Get the t_pos value of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point
        t_pos: float
            t_pos value of the desired cost point

        Returns
        -------
        empty
        """
        self.costs.nodes[(x, y, t)]['t_pos'] = t_pos

    def getTPos(self, x, y, t):
        """
        Get the t_pos of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            t_pos of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['t_pos']

    def toVizMsg(self, cost_grid_params):
        # local variables
        n_t = cost_grid_params.get('n_t')
        n_x = cost_grid_params.get('n_x')
        n_y = cost_grid_params.get('n_y')
        id = 1
        marker_array_msg = MarkerArray()

        # iterate over all grids
        # TODO make time dependent and add to sampler to update most relevant layer - currently only the first time layer is implemented...
        # for k in range(n_t):
        k = 0
        for i in range(n_x):
            for j in range(n_y):
                cost = self.costs.nodes[(i, j, k)]['node_weight']

                # visualizations of each cost grid element
                marker = Marker()
                marker.id = id
                id += 1
                marker.ns = 'cost_grid'
                marker.type = Marker.CYLINDER
                marker.header.frame_id = "/map"
                marker.action = Marker.ADD
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = cost / 10.0
                marker.color.a = 0.5
                marker.color.r = cost
                marker.color.g = 0.1
                marker.color.b = 0.1
                marker.pose.position.x = self.getXWorld(i,j,k)
                marker.pose.position.y = self.getYWorld(i,j,k)
                marker.pose.position.z = - 0.05 + (self.getTPos(i,j,k) + cost / 2.0)/10

                marker_array_msg.markers.append(marker)

        return marker_array_msg
