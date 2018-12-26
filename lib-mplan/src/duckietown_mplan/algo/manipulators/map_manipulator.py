__all__ = [
    'MapManipulator',
]


import rospy
import numpy as np
import os
import tf
import math
import random
import mplan_msgs.msg as mpmsg
from visualization_msgs.msg import Marker

from duckietown_mplan.algo.containers import Obstacle

class MapManipulator:
    """Handy methods for map manipulation"""

    def __init__(self, tile_size):
        """
        initialize map manipulator object

        Parameters
        ----------
        tile_size: float
            length of a tile side in meters

        Returns
        -------
        empty
        """
        self.tile_size = tile_size
        self.always_go_straight = rospy.get_param('always_go_straight')

    def __del__(self):
        """
        destructor

        Parameters
        ----------
        empty

        Returns
        -------
        empty
        """
        pass

    def mapToPositiveAngle(self, angle):
        """
        map an angle to [0, 2pi]

        Parameters
        ----------
        angle: float
            an angle in radian in [-inf, inf]

        Returns
        -------
        float: the mapped angle in radian
        """
        if angle >= 2*math.pi:
            angle -= 2*math.pi
        elif angle < 0:
            angle += 2*math.pi
        return angle

    def parseMapToTileList(self, map):
        """
        parse a map to a list of tiles

        Parameters
        ----------
        map: MarkerArray
            an array of markers as published by duckietown_vizualisation

        Returns
        -------
        list of dicts: the list of tiles where every tile is a dict with entries
        position, type and entry angle
        """
        # find the closest tile and return its position and orientation
        tiles = []
        for elem in map.markers:
            if elem.ns == 'tiles':
                name = os.path.basename(os.path.normpath(elem.mesh_resource))
                if name != 'asphalt.dae':
                    quaternion = [elem.pose.orientation.x, elem.pose.orientation.y, elem.pose.orientation.z, elem.pose.orientation.w]
                    yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
                    yaw = self.mapToPositiveAngle(yaw)
                    tiles.append({'position' : [elem.pose.position.x, elem.pose.position.y, yaw], 'type' : name, 'entry_angle' : yaw})
        return tiles

    def findClosestTile(self, x, y, tile_list):
        """
        find the tile which lies closest to a point x,y

        Parameters
        ----------
        x: float
            the x position of the point
        y: float
            the y position of the point
        tile_list: list of dicts
            a list containing all tiles as produced by MapManipulator.parseMapToTileList()

        Returns
        -------
        dict: the closest tile
        """
        tile_positions = [elem['position'] for elem in tile_list]
        tile_positions = np.asarray(tile_positions)
        deltas = tile_positions[:,:2] - [x, y]
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return tile_list[np.argmin(dist_2)]

    def distToTile(self, x, y, tile):
        """
        get the distance from a point x,y to the midpoint of a specific tile

        Parameters
        ----------
        x: float
            the x position of the point
        y: float
            the y position of the point
        tile: dict
            the tile to which the distance is desired

        Returns
        -------
        float: the distance
        """
        return ((x-tile['position'][0])**2 + (y-tile['position'][1])**2)**0.5

    def distVecFromTile(self, x, y, tile):
        """
        get the distance vector from a point x,y to the midpoint of a specific tile

        Parameters
        ----------
        x: float
            the x position of the point
        y: float
            the y position of the point
        tile: dict
            the tile to which the distance is desired

        Returns
        -------
        np.array: the 3x1 distance vector in world coordinates
        """
        return np.asarray([x-tile['position'][0], y-tile['position'][1]])

    def getUnitVecFromTheta(self, theta, length=1):
        """
        get a unit vector in the xy plane pointing in the direction of theta

        Parameters
        ----------
        x: theta
            angle in radian corresponding to the direction of the desired vector
        y: length
            length of the returned vector

        Returns
        -------
        np.array: the 3x1 vector with direction of theta
        """
        return np.asarray([math.cos(theta), math.sin(theta), 0])*length

    def getNextTile(self, tile, tile_list):
        """
        get the next tile to which the street on the current tile will lead to.
        If the current tile is an intersection the next tile will be chosen at
        random.

        Parameters
        ----------
        tile: dict
            the tile to which the distance is desired
        tile_list: list of dicts
            a list containing all tiles as produced by MapManipulator.parseMapToTileList()

        Returns
        -------
        dict: the next tile
        """
        delta_theta = tile['entry_angle']-tile['position'][2]
        delta_theta = self.mapToPositiveAngle(delta_theta)

        xy = [0,0,0]
        next_entry_angle = 0

        if tile['type']=='straight.dae':
            next_entry_angle = tile['entry_angle']

        elif tile['type']=='curve_left.dae':
            if delta_theta == 0:
                next_entry_angle = tile['entry_angle']+math.pi/2
            if delta_theta == math.pi*3/2:
                next_entry_angle = tile['entry_angle']-math.pi/2

        elif tile['type']=='curve_right.dae':
            if delta_theta == math.pi*2/2:
                next_entry_angle = tile['entry_angle']-math.pi/2
            if delta_theta == math.pi*3/2:
                next_entry_angle = tile['entry_angle']+math.pi/2

        elif tile['type']=='4way.dae':
            next_entry_angle = tile['entry_angle'] + random.randint(-1,1)*math.pi/2*int(self.always_go_straight)

        elif tile['type']=='3way_left.dae':
            if delta_theta == 0:
                next_entry_angle = tile['entry_angle'] + random.randint(0,1)*math.pi/2*int(self.always_go_straight)
            if delta_theta == math.pi:
                next_entry_angle = tile['entry_angle'] + random.randint(-1,0)*math.pi/2*int(self.always_go_straight)
            if delta_theta == math.pi*3/2:
                next_entry_angle = tile['entry_angle'] + (random.randint(0,1)-1.0/2)*math.pi*int(self.always_go_straight)

        elif tile['type']=='3way_right.dae':
            if delta_theta == 0:
                next_entry_angle = tile['entry_angle'] + random.randint(-1,0)*math.pi/2
            if delta_theta == math.pi/2:
                next_entry_angle = tile['entry_angle'] + (random.randint(0,1)-1/2)*math.pi
            if delta_theta == math.pi:
                next_entry_angle = tile['entry_angle'] + random.randint(0,1)*math.pi/2
        else:
            print('unforeseen tile entry angle - could not find next')

        xy = tile['position']+self.getUnitVecFromTheta(next_entry_angle, self.tile_size)
        next_tile = self.findClosestTile(xy[0], xy[1], tile_list)
        next_tile['entry_angle'] =  self.mapToPositiveAngle(next_entry_angle)
        return next_tile

    def getMarker(self, marker_id, tile):
        """
        convert the tile to an arrow representing the entry angle of the
        duckiebot into the tile's street. This arrow can be visualized in rviz

        Parameters
        ----------
        marker_id: int
            the id for the marker
        tile: dict
            the tile to be converted to a arrow marker

        Returns
        -------
        Marker: the arrow marker with the correct position
        """

        marker = Marker()

        marker.header.frame_id = "/map"
        marker.id = marker_id
        marker.ns = "tile_markers"

        marker.type = marker.ARROW
        marker.action = marker.ADD

        marker.pose.position.x = tile['position'][0]
        marker.pose.position.y = tile['position'][1]
        marker.pose.position.z = 0.2

        marker.scale.x = 0.2
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        q = tf.transformations.quaternion_from_euler(0, 0, tile['entry_angle'])
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0/marker_id
        marker.color.a = 1.0

        return marker

    def rotation_matrix(self, theta, axis=[0,0,1]):
        """
        get the rotation matrix corresponding to a counterclockwise rotation of
        theta around a specific axis

        Parameters
        ----------
        theta: float
            the desired rotation angle in radian
        axis: 3x1 list
            the vector around whicht the rotation should be induced

        Returns
        -------
        np.array: 3x3 the rotation matrix
        """
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                         [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                         [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    def getCostGridOrigin(self, tile_current, tile_next, actor):
        """
        get the position and orientation of the point at which the cost grid's
        origin should be placed. Lattitudewise this position will always be in
        the middle of the street, longitudewise the position will be the
        projection of the duckiebots position on to the middle-line of the
        street. The orientation will always be according to the streeght
        orientation if it is a straight or according to the desired path
        curvature if the street is a curve or intersection.

        Parameters
        ----------
        tile_current: dict
            the tile on which the duckiebot is currently on_shut
        tile_next: dict
            the tile on which the duckiebot will be next after leaving the
            current tile
        actor: class obstacle
            obstacle representing the duckiebot which is controlled by the
            obstavoid algorithm

        Returns
        -------
        list: 3x1 the x, y, theta position of the cost grid
        """
        type = tile_current['type']
        entry_angle_difference = self.mapToPositiveAngle(tile_next['entry_angle']-tile_current['entry_angle'])

        if entry_angle_difference == 0: #straight

            d = self.distVecFromTile(actor.x, actor.y, tile_current)
            e = self.getUnitVecFromTheta(tile_current['entry_angle'])[:2]
            offset = np.dot(d, e)*e
            cost_grid_origin = [tile_current['position'][0]+offset[0], tile_current['position'][1]+offset[1], tile_next['entry_angle']]

        elif entry_angle_difference == math.pi/2: #left curve

            # tile position in world frame
            w_P_t = tile_current['position'][:2]

            # duckiebot positon in world frame
            w_P_d = np.array([actor.x, actor.y])

            # corner position in tile frame
            t_P_c = np.dot(self.rotation_matrix(tile_current['entry_angle']), np.array([-self.tile_size/2, self.tile_size/2, 0]))[:2]

            # corner position in world frame
            w_P_c = w_P_t + t_P_c

            # unit radius vector
            radius = w_P_d - w_P_c
            unit_radius = radius / np.linalg.norm(radius)

            # get positional cost_grid_origin
            cost_grid_origin = w_P_c + self.tile_size/2 * unit_radius

            # get theta angle of cost_grid
            theta = self.mapToPositiveAngle(math.atan2(unit_radius[1], unit_radius[0])+math.pi/2)

            cost_grid_origin = [cost_grid_origin[0], cost_grid_origin[1], theta]

        elif entry_angle_difference == 3*math.pi/2: #left curve

            # tile position in world frame
            w_P_t = tile_current['position'][:2]

            # duckiebot positon in world frame
            w_P_d = np.array([actor.x, actor.y])

            # corner position in tile frame
            t_P_c = np.dot(self.rotation_matrix(tile_current['entry_angle']), np.array([-self.tile_size/2, -self.tile_size/2, 0]))[:2]

            # corner position in world frame
            w_P_c = w_P_t + t_P_c

            # unit radius vector
            radius = w_P_d - w_P_c
            unit_radius = radius / np.linalg.norm(radius)

            # get positional cost_grid_origin
            cost_grid_origin = w_P_c + self.tile_size/2 * unit_radius

            # get theta angle of cost_grid
            theta = self.mapToPositiveAngle(math.atan2(unit_radius[1], unit_radius[0])-math.pi/2)

            cost_grid_origin = [cost_grid_origin[0], cost_grid_origin[1], theta]

        return cost_grid_origin

    def getDistToCenterline(self, tile, x, y):
        """
        get the distance from point x,y to the centerline of a specific tile

        Parameters
        ----------
        tile: dict
            the tile to which's centerline the distance should be calculated
        x: float
            the x position of the point
        y: float
            the y position of the point

        Returns
        -------
        float: the distance
        """
        d = self.distVecFromTile(x, y, tile)
        e = self.getUnitVecFromTheta(tile['entry_angle'] + math.pi / 2)[:2]

        dist_to_centerline = np.dot(d, e)
        return dist_to_centerline
