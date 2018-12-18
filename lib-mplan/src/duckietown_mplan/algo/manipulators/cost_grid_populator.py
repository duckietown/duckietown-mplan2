__all__ = [
    'CostGridPopulator',
]


from duckietown_mplan.algo.containers import Obstacle
from duckietown_mplan.algo.containers import CostGrid
import math
import numpy as np
import sympy as sp
import rospy

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class CostGridPopulator:
    """Populates the cost grid from the existing objects"""


    def __init__(self, cost_grid_params, max_actor_vel):
        """
        initialize cost_grid object

        Parameters
        ----------
        empty

        Returns
        -------
        empty
        """
        # initialize cost helper functions
        self.push_fwd_frac = rospy.get_param('cost_function/push_fwd/frac')
        self.street_bound_frac = rospy.get_param('cost_function/street_bound/frac')
        self.obst_avoid_frac = rospy.get_param('cost_function/obstacles/frac')
        self.init_fixed_fun(cost_grid_params, max_actor_vel)

        # create cost_grid object
        self.cost_grid = CostGrid()

        #  fill graph with nodes
        self.fillGraph(cost_grid_params)


    def __del__(self):
        pass

    def init_push_fwd_fun(self, cost_grid_params, max_actor_vel):
            """
            Creates a symbolic function for the push forward component of the
            cost function

            Parameters
            ----------
            cost_grid_params: dictionary
            a dictionary containing the cost grid dimensions
            max_actor_vel: float
            target velocity of duckiebot
            Returns
            -------
            n.a.
            """

            # TODO
            # - make adaptive for curved streets
            # - try simpler function, 5th order might be an overkill
            # - add parameters to centralized file

            # local parameters needed
            dt = cost_grid_params.get('dt')
            n_t = cost_grid_params.get('n_t')

            # cost for standing still
            push_fwd_stand_cost = rospy.get_param('cost_function/push_fwd/stand_cost')
            # 10% above max speed, only for viz
            push_fwd_allow_speed_fract = rospy.get_param('cost_function/push_fwd/allow_speed')

            # Parameters for cost quintic
            x_opt = dt*n_t* max_actor_vel     # optimal position - max speed for the whole time interval
            x_max = (1.0 + push_fwd_allow_speed_fract) *dt*n_t * max_actor_vel     # distance after which cost shpuld go to infinity - vehicle can not physically drive faster than max speed

            # one dimensional 5th order polynomial is used to model the cake
            x = sp.Symbol('x')
            y = sp.Symbol('y')
            a = sp.Symbol('a')
            b = sp.Symbol('b')
            c = sp.Symbol('c')
            d = sp.Symbol('d')
            e = sp.Symbol('e')
            f = sp.Symbol('f')
            g = sp.Function('g')
            g_dot = sp.Function('g_dot')
            g_dot_dot = sp.Function('g_dot_dot')
            g = a * x**5 + b * x**4 + c * x**3 + d * x**2+ e * x + f
            g_dot = sp.diff(g,x)
            g_dot_dot = sp.diff(g_dot,x)

            # equations:
            eq1 = sp.Eq(g.subs(x,0), push_fwd_stand_cost) # penalize not moving
            eq2 = sp.Eq(g_dot.subs(x,0), 0) #
            eq3 = sp.Eq(g_dot_dot.subs(x,0), -0.5) #
            eq4 = sp.Eq(g.subs(x,x_opt), 0) # give zero cost when driving with optimal velocity
            eq5 = sp.Eq(g_dot.subs(x,x_opt), 0) #
            eq6 = sp.Eq(g_dot_dot.subs(x,x_opt), 1)

            # solve system of equations
            results = sp.solve([eq1, eq2, eq3, eq4, eq5, eq6],[a,b,c,d,e,f])
            a_num = results[a]
            b_num = results[b]
            c_num = results[c]
            d_num = results[d]
            e_num = results[e]
            f_num = results[f]
            push_fwd_fun = sp.Function('push_fwd_fun')
            push_fwd_fun = g.subs([(a,a_num), (b,b_num), (c,c_num), (d,d_num), (e,e_num), (f,f_num)])

            # DEBUG do not delete
            # print push_fwd_fun
            # print "x_opt: {x_opt}, x_max: {x_max}".format(x_opt = x_opt,x_max = x_max)
            # sp.plotting.plot3d(push_fwd_fun, (x, 0, x_max), (y, -0.2, 0.2), xlim=[-0.1, x_max*1.5], ylim=[-0.3,0.3])

            # safe
            self.push_fwd_fun = push_fwd_fun
            self.push_fwd_fun_type = "straight"

            pass

    def init_street_bound_fun(self, cost_grid_params, max_actor_vel):
                """
                Creates a symbolic function for the street bound and correct lane
                following component of the cost function

                Parameters
                ----------
                cost_grid_params: dictionary
                a dictionary containing the cost grid dimensions
                max_actor_vel: float
                target velocity of duckiebot
                Returns
                -------
                n.a.
                """

                # TODO
                # - make adaptive for curved streets
                # - try simpler function, 6th order might be an overkill
                # - add parameters to centralized file

                # local parameters needed
                dy = cost_grid_params.get('dy')
                n_y = cost_grid_params.get('n_y')

                # parameters needed, add to global file TODO
                street_bound_cost_max = 1

                # Parameters for 4th order polynomial
                y_opt_lane = -(n_y-1)/4.0*dy
                y_other_lane = -y_opt_lane
                y_left_boarder = -(n_y-1)/2.0*dy
                y_right_boarder = (n_y-1)/2.0*dy

                # one dimensional 6th order polynomial is used to model the straight street
                x = sp.Symbol('x')
                y = sp.Symbol('y')
                a = sp.Symbol('a')
                b = sp.Symbol('b')
                c = sp.Symbol('c')
                d = sp.Symbol('d')
                e = sp.Symbol('e')
                f = sp.Symbol('f')
                g = sp.Symbol('g')
                params = [a,b,c,d,e,f,g]
                h = sp.Function('h')
                h_dot = sp.Function('h_dot')
                h_dot_dot = sp.Function('h_dot_dot')
                h = a * y**6 + b * y**5 + c * y**4 + d * y**3 + e * y**2 + f * y + g
                h_dot = sp.diff(h,y)
                h_dot_dot = sp.diff(h_dot,y)

                # equations:
                eq1 = sp.Eq(h.subs(y,y_opt_lane), 0) # diff should be 0 at beginning
                eq2 = sp.Eq(h_dot.subs(y,y_opt_lane), 0) # diff should be 0 at beginning
                eq3 = sp.Eq(h.subs(y,y_other_lane), 1) # 0 cost at optimal distance
                eq4 = sp.Eq(h_dot.subs(y,y_other_lane), 0) # diff should be 0 at beginning
                eq5 = sp.Eq(h.subs(y,0), 0.5) # diff should be 0 at beginning
                eq6 = sp.Eq(h.subs(y,y_left_boarder), 1) # 0 cost at optimal distance
                eq7 = sp.Eq(h.subs(y,y_right_boarder), 2) # 0 cost at optimal distance

                # solve system of equations
                results = sp.solve([eq1, eq2, eq3, eq4, eq5, eq6, eq7],[a,b,c,d,e,f,g])
                a_num = results[a]
                b_num = results[b]
                c_num = results[c]
                d_num = results[d]
                e_num = results[e]
                f_num = results[f]
                g_num = results[g]
                street_bound_cost_fun = sp.Function('street_bound_cost_fun')
                street_bound_cost_fun = h.subs([(a,a_num), (b,b_num), (c,c_num), (d,d_num), (e,e_num), (f,f_num), (g,g_num)])

                # DEBUG VISUALIZATIONS
                # print "y_opt_lane: {y_opt_lane}, y_other_lane: {y_other_lane}, y_left_boarder: {y_left_boarder}, y_right_boarder: {y_right_boarder}".format(y_opt_lane = y_opt_lane, y_other_lane = y_other_lane, y_left_boarder = y_left_boarder,y_right_boarder = y_right_boarder)
                # print street_bound_cost_fun
                # sp.plotting.plot3d(street_bound_cost_fun, (x, 0, 1.5), (y, y_left_boarder, y_right_boarder), xlim=[-0.1,1.5], ylim=[y_left_boarder-0.1,y_right_boarder+0.1])

                self.street_bound_fun = street_bound_cost_fun
                self.street_bound_fun_type = "straight"

                pass

    def init_fixed_fun(self, cost_grid_params, max_actor_vel):

        """
        Creates a function for the fixed cost determined by the environment

        Parameters
        ----------
        cost_grid_params: dictionary
        a dictionary containing the cost grid dimensions
        max_actor_vel: float
        target velocity of duckiebot
        Returns
        -------
        n.a.
        """

        # Initialise Constant Functions
        self.init_push_fwd_fun(cost_grid_params, max_actor_vel)
        self.init_street_bound_fun(cost_grid_params, max_actor_vel)

        # create symbolic objects
        x = sp.Symbol('x')
        y = sp.Symbol('y')
        t = sp.Symbol('t')
        total_fun = sp.Function('total_fun')

        # combine the constant functions
        total_fun = self.push_fwd_frac * self.push_fwd_fun + self.street_bound_frac * self.street_bound_fun

        # create a function for fast execution
        self.fixed_cost = sp.lambdify([x,y,t], total_fun)

    def fillGraph(self, cost_grid_params):
        """
        Fill the networkx graph of the cost grid with nodes

        Parameters
        ----------
        cost_grid_params: dictionary
            a python dictionary containing the size and resolution of the cost
            grid. Entries are 'n_t', 'n_x', 'n_y', 'dt', 'dx', 'dy'
        Returns
        -------

        """
        # add nodes to cost_grid object
        for k in range(cost_grid_params.get('n_t')):
            for i in range(cost_grid_params.get('n_x')):
                for j in range(cost_grid_params.get('n_y')):
                    x_pos=round(i*cost_grid_params.get('dx'),2)
                    y_pos=round((j-(cost_grid_params.get('n_y')-1)/2.0)*cost_grid_params.get('dy'), 2) # for centered coordinate system
                    t_pos=k*cost_grid_params.get('dt')
                    self.cost_grid.costs.add_node((i, j, k), x_pos=x_pos, x_world=x_pos, y_pos=y_pos, y_world=y_pos, t_pos=t_pos, node_weight=0.0)

    def connectGraph(self, graph, actor_x, actor_y, cost_grid_params, max_actor_vel, dist_to_centerline):
        """
        connect the graph of an obstacle grid

        Parameters
        ----------
        graph : networkx.Graph
            the graph to be connected
        actor : containers.Obstacle
            state object of the actor
        max_actor_vel : float
            the maximum velocity in [m/s] of a duckiebot

        Returns
        -------
        networkx.Graph : the connected graph
        """

        def saturator(arg, s_min, s_max):
            if arg < s_min:
                res = s_min
            elif arg > s_max:
                res = s_max
            else:
                res = arg
            return res

        # saturate distance to centerline
        sat_dist_to_centerline = saturator(dist_to_centerline, -cost_grid_params.get('dy'), cost_grid_params.get('dy'))

        # connect start node (find closest node index to current actor position) to first layer (layer 0)
        k_0 = 0
        i_0 = 0
        j_0 = int(round(sat_dist_to_centerline/cost_grid_params.get('dy')+(cost_grid_params.get('n_y')-1)/2.0))
        self.cost_grid.costs.add_weighted_edges_from([('S', (i_0,j_0,k_0), 0.0)])

        # connect layers from layer 0 to layer n_t-1
        # initialize first iteration
        k_n = k_0
        i_n = i_0
        j_n = j_0
        active_nodes = [(i_n, j_n, k_n)]

        # instantiate mask
        mask = [(0,0), (1,0), (-1,0), (0,1), (0, -1), (1,1), (-1,1), (1, -1), (-1, -1), (0,2), (0,-2)]

        # connect end node to last layer (layer n_t-1)
        k = cost_grid_params.get('n_t') - 1
        for i in range(cost_grid_params.get('n_x')):
            for j in range(cost_grid_params.get('n_y')):
                self.cost_grid.costs.add_weighted_edges_from([((i,j,k), 'E', 0.0)])

        # iterate
        for k in range(cost_grid_params.get('n_t')-1):
            next_nodes = []
            for current_node in active_nodes:
                i_n = current_node[0]
                j_n = current_node[1]
                k_n = current_node[2]
                for mask_element in mask:
                    mask_x = i_n + mask_element[0]
                    mask_y = j_n + mask_element[1]
                    curr_node = (i_n, j_n, k_n)
                    next_node = (mask_x, mask_y, k_n + 1)
                    if mask_x>=0 and mask_x<=(cost_grid_params.get('n_x')-1) and mask_y>=0 and mask_y<=(cost_grid_params.get('n_y')-1):
                        self.cost_grid.costs.add_weighted_edges_from([(curr_node, next_node, self.getEdgeCost(curr_node, next_node))])
                        if next_node not in next_nodes:
                            next_nodes.append((mask_x, mask_y, k_n + 1))
            active_nodes = next_nodes

    def populate(self, actor_position, list_of_obstacles, street_obstructions, cost_grid_params, max_actor_vel, origin, dist_to_centerline):
        """
        Create a cost grid and populate it according to the obstacles

        Parameters
        ----------
        actor_position : undefinde position object #TODO
            an object containing the position and speed of acting duckiebot
        list_of_obstacles: obstacle[]
            a list of obstacle objects known to be around - other duckiebots
        street_obstructions: obstacle list
            an obstacle object known to be around - not a duckiebot
        cost_grid_params: dictionary
            a python dictionary containing the size and resolution of the cost
            grid. Entries are 'n_t', 'n_x', 'n_y', 'dt', 'dx', 'dy'
        max_actor_vel: float
            the maximum velocity in [m/s] of a duckiebot
        marker_array: visualization_msgs/Marker.msg
            containing all the grid information for plotting

        Returns
        -------
        CostGrid : a cost grid where each grid point has an assigned cost
        """
        # local variables
        n_t = cost_grid_params.get('n_t')
        n_x = cost_grid_params.get('n_x')
        n_y = cost_grid_params.get('n_y')
        id = 1

        # TODO fill these with mid position of street
        # transform between cost-grid frame and world frame. These are cost grid origin positions in world frame
        x_origin = origin[0]
        y_origin = origin[1]
        theta_origin = origin[2]

        # add weighted nodes to cost_grid object
        for k in range(n_t):
            for i in range(n_x):
                for j in range(n_y):
                    # get temporal and spatial position of current node
                    x = self.cost_grid.getXPos(i,j,k)
                    y = self.cost_grid.getYPos(i,j,k)
                    t = self.cost_grid.getTPos(i,j,k)

                    # calculate cost for node
                    cost = self.calculateCost(x, y, t, list_of_obstacles, street_obstructions, actor_position)

                    # set cost of node and world position of node, world position of node is actor position + node position in cost grid frame
                    self.cost_grid.setCost(i, j, k, cost)
                    self.cost_grid.setXWorld(i, j, k, x_origin + math.cos(theta_origin)*x-math.sin(theta_origin)*y)
                    self.cost_grid.setYWorld(i, j, k, y_origin + math.sin(theta_origin)*x+math.cos(theta_origin)*y)

        # add weighted edges to graph
        self.connectGraph(self.cost_grid.costs, actor_position.x, actor_position.y, cost_grid_params, max_actor_vel, dist_to_centerline)
        self.cost_grid.populated = True
        return self.cost_grid

    def calculateCost(self, x_df, y_df, t_df, obstacle_list, street_obstructions, actor):
        """
        return the value of the costfunction at a specific time point

        Parameters
        ----------
        x_rw : float
            x position of the requested cost value in the duckie frame
        y_rw : float
            y position of the requested cost value in the duckie frame
        t_rw : float
            time of requested cost value in the duckie frame
        obstacle_list (TODO): containers.Obstacle[]
            list of obstacles state objects

        Returns
        -------
        float : the requested cost
        """

        # get cost of straight line & forward cost
        cost = self.fixed_cost(x_df, y_df, t_df)

        # TODO: generalize the transfrom Function --> is there already a funciton that can do this transformation?
        # convert duckie frame into real_world frame
        x_rwf = np.cos(actor.theta) * x_df - np.sin(actor.theta) * y_df + actor.x
        y_rwf = np.sin(actor.theta) * x_df + np.cos(actor.theta) * y_df + actor.y
        t_rwf = t_df

        # add the cost of every duckiebot obstacle
        for elem in obstacle_list:
            cost += self.obst_avoid_frac * elem.getCost(x_rwf,y_rwf,t_rwf, elem.x, elem.y, elem.x_dot, elem.y_dot, elem.radius)

        # add the cost of very street obstruction
        for elem in street_obstructions:
            cost += elem.getCost(x_rwf,y_rwf,t_rwf, elem.x, elem.y, elem.x_dot, elem.y_dot, elem.radius)

        return cost

    def getEdgeCost(self, curr_node, next_node):
        """
        defines the relation from edge length to edge cost
        (currently proportional to the euclidean distance)

        Parameters
        ----------
        curr_node: tuple(3)
            the current node
        next_node: tuple(3)
            the next node

        Returns
        -------
        float : the requested cost
        """
        curr_delta_x = self.cost_grid.getXPos(next_node[0], next_node[1], next_node[2])-self.cost_grid.getXPos(curr_node[0], curr_node[1], curr_node[2])

        curr_delta_y = self.cost_grid.getYPos(next_node[0], next_node[1], next_node[2])-self.cost_grid.getYPos(curr_node[0], curr_node[1], curr_node[2])

        norm_factor = 0.0; # depends on the order of magnitude of the node cost function
        distance = norm_factor * math.sqrt(curr_delta_x**2 + curr_delta_y**2)

        return distance
