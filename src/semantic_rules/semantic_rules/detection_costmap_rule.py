# -*- coding: utf-8 -*-
import math
import warnings
import uuid
import rclpy
import numpy as np
from rclpy import Parameter
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point, Vector3, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray  # TODO: ensure this is properly imported
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# alternatively use a simple collections.deque
from semantic_rules.circular_time_buffer import CircularTimeBuffer


class SemanticCostmapRule(Node):
    _rule_type_choices = ['parameters', 'code']
    _cost_type_choices = ['velocity_falloff', 'direction_falloff', 'custom']
    _direction_type_choices = ['front', 'sector', 'all_sides', 'gradient', 'custom']
    # TODO: abs_percentage is effectively linear - remove?
    _falloff_type_choices = ['rel_percentage', 'abs_percentage', 'linear', 'custom']
    _semantic_classification = ['binary', 'fast_slow', 'custom']
    # both strings and id-s will be checked for assigning category, so feel free to add classes based on either
    _dynamic_objects = {} # (class: category) - dict gives the fastest lookup (?) and doesn't need 3 containers

    testing = True  # use testing loggers

    def _validate_params(self):
        # type checks
        if self.rule_type not in self._rule_type_choices:
            raise IOError(f"Selected rule type not implemented. Possible choices are {self._rule_type_choices}")

        if self.cost_type not in self._cost_type_choices:
            raise IOError(f"Selected cost type not implemented. Possible choices are {self._cost_type_choices}")

        if self.direction_type not in self._direction_type_choices:
            raise IOError(f"Selected direction type not implemented. Possible choices are {self._direction_type_choices}")

        if self.falloff_type not in self._falloff_type_choices:
            raise IOError(f"Selected direction type not implemented. Possible choices are {self._direction_type_choices}")
        # other checks
        # if self.min_range > self.max_range:
        #     raise IOError(f"Please provide a velocity_segments bigger than the min_range. No point in creating empty costmaps."
        #                   f" Min and max range may be equal.")

    def _check_semantics(self):
        self.semantic_classification = self.get_parameter("semantic_classification")._value
        if self.semantic_classification not in self._semantic_classification:
            raise IOError(
                f"Selected classification not implemented. Please use 'custom' semantic classification (and prefably cost_type) for more flexible semantic data usage."
                f" Possible semantic classification choices are {self._semantic_classification}. Binary uses static and dynamic objects. fast_slow adds fast and slow categories (totalling 4)")
        # TODO: separate check and set? do we want to allow changing classification while running?
        user_dynamic_objects = self.get_parameter("dynamic_objects")._value
        user_fast_objects = self.get_parameter("fast_objects")._value
        user_slow_objects = self.get_parameter("slow_objects")._value
        if self.semantic_classification == 'custom':
            if self.cost_type != 'custom':  # separate if in case we actually do decide to implement this
                raise NotImplementedError("We haven't implemented builtin cost-type for custom semantic classification "
                                          "and we probably never will. "
                                          "Please use 'custom' cost type when using 'custom' semantic classification")
            if self.testing:
                self.get_logger().info(f"Using custom semantic classification")
        # TODO: it would be better to read from file...
        for c in user_dynamic_objects:
            self._dynamic_objects[c] = 'moving'

        if self.semantic_classification == 'fast_slow':
            for f in user_fast_objects:
                self._dynamic_objects[f] = 'fast'
            for s in user_slow_objects:
                self._dynamic_objects[s] = 'slow'

        if self.testing:
            self.get_logger().info(f"Using following regular dynamic object classes: {self._dynamic_objects}")

    def _check_params(self):
        self.rule_type = self.get_parameter("rule_type")._value
        self.direction_type = self.get_parameter("direction_type")._value
        self.cost_type = self.get_parameter("cost_type")._value
        self.reverse_falloff = self.get_parameter("reverse_falloff").get_parameter_value().bool_value
        self.velocity_segments = self.get_parameter("velocity_segments").get_parameter_value().integer_value
        self.min_range = self.get_parameter("min_range").get_parameter_value().integer_value
        self.max_range = self.min_range + self.velocity_segments
        self.base_cost = self.get_parameter("base_cost").get_parameter_value().double_value
        self.falloff = self.get_parameter("falloff").get_parameter_value().double_value
        self.falloff_type = self.get_parameter("falloff_type")._value
        self._validate_params()
        if self.testing:
            self.get_logger().info(
                F"We have set the following parameters: "
                F"{[self.rule_type, self.direction_type, self.cost_type, self.velocity_segments, self.min_range]}"
            )
        self._check_semantics()
        self.velocity_duration = self.get_parameter("velocity_duration").get_parameter_value().double_value
        # TODO: We currently don't check all costmap params here - costmap_frame!
        self.resolution = self.get_parameter("resolution").get_parameter_value().double_value
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        # if self.rule_type != 'custom' and (self.max_range-self.min_range) <= 1 / self.resolution:
        #     raise ValueError("Resolution must allow costs to fit in a unit vector. "
    #                      "That means you have to set a resolution: resolution >= 1 / (max_range - min_range)")
        if self.rule_type != 'custom' and (self.max_range-self.min_range) != 1 / self.resolution and self.cost_type == 'direction_falloff':
            warnings.warn("""
            Resolution should allow costs to fit in a unit vector. 
            Setting a resolution smaller than range difference results in an unintuitive
            velocity usage. It is recommended to set params: resolution == 1 / (max_range - min_range)
            """)

    def __init__(self):
        super().__init__('detection_costmap_rule')
        # TODO: evaluate whether or not to save parameters as class variables
        self.declare_parameters(
            namespace='',
            parameters=[
                # input params
                # ('detection_topic', "dynamic_object"),  # TODO: consider - remapping is better then using parameters?
                ('sequence', 1),  # TODO: implement sequence
                # cost calculation params
                ('rule_type', "parameters"),
                ('direction_type', "all_sides"),
                ('cost_type', "velocity_falloff"),
                ('reverse_falloff', False),
                ('falloff_type', "linear"),
                ('base_cost', 100.0),
                ('falloff', 0.5),
                ('min_range', 0),  # the start point distance (on costmap) from origin for a velocity of unit vector
                ('velocity_segments', 10),  # number of costs to assign to a velocity.
                ('velocity_duration', 1.0),
                # the duration for which the costs are calculated. Effectively velocity multiplier
                # TODO: semantic classification params may all be removed, as it is handled by the converter node
                # semantic classification params
                ('semantic_classification', 'binary'),
                ('dynamic_objects', ['person', 'human', 'cat', 'dog', 'car', 'truck', 'bus']),
                ('fast_objects', ['car', 'truck', 'bus']),
                ('slow_objects', ['person', 'human']),
                ('fast_velocity_segments', 20),
                ('slow_velocity_segments', 5),
                ('fast_falloff', 0.25),
                ('slow_falloff', 0.75),
                ('fast_base_cost', Parameter.Type.DOUBLE),
                ('slow_base_cost', Parameter.Type.DOUBLE),
                # TODO: should this be handled solely in the Hungarian tracker node?
                # buffer params - buffers are for 1) obstacles 2) costmap or 3) global map
                # we use integer buffer params. see CircularTimeBuffer for more precise options
                ('idleness_falloff', 0.5), # this is actually cost calculation param, but useless if no buffer is used
                ('idleness_range', 0), # This is actually mostly for 'custom' cost_type - ignoring circular movement  # TODO: param descriptor for float values
                ('use_obstacle_buffer', False),
                ('obs_buffer_size', 3),
                ('obs_buffer_period', 5),
                ('use_costmap_buffer', False),
                ('map_buffer_size', 3),
                ('map_buffer_period', 5),
                ('use_global_buffer', False),
                ('global_buffer_size', 3),
                ('global_buffer_period', 5),
                # general costmap params
                ('costmap_frame', 'map'),
                ('resolution', 0.1),
                ('width', 84),
                ('height', 91),
                # ('map_load_time', [sec, ns]),
                # ('origin', [position, orientation]),
                # custom costmap params
                ('vel_filter', [0.1, 20.0]),  # [min, max]
                ('height_filter', [-2.0, 2.0]),  # TODO: This is duplicated in the Hungarian tracker?
                ('cost_filter', Parameter.Type.DOUBLE),
                # other custom params?
                # ('global_frame', "camera_link"),  # - what was this??? # TODO: this is definitely necessary - implement
                ('process_noise_cov', [2., 2., 0.5]),
                ('top_down', False),
                ('death_threshold', 3),
                ('measurement_noise_cov', [1., 1., 1.]),
                ('error_cov_post', [1., 1., 1., 10., 10., 10.]),
            ])
        # self.detection_topic = self.get_parameter("detection_topic")._value

        # TODO: make topic_name dynamic based on param (detection_publisher -> self.detection_topic)
        #  although nav2 cpp costmap takes the topicname as class init input...?
        """ PUBLISHERS """
        self.costmap_publisher = self.create_publisher(OccupancyGrid, 'costmap_publisher', 10)
        # self.dynamic_tf_pub - declared in """ setup tf related """

        """ SUBSCRIBERS """
        self.tracker_sub = self.create_subscription(
            ObstacleArray,
            'tracking',
            self.track_callback,
            10)

        # current data NB! these contain only the data of the corresponding message types
        # For full messages use buffers
        self.obstacles = []
        self.costmap = OccupancyGrid()
        self.global_map = []
        self.sec = 0
        self.nanosec = 0

        # Buffers for storing data
        self.buffer_accept_none = False
        # obstacle buffer (default: Enabled) - this is in the local environment, local map
        use_obstacle_buffer = self.get_parameter("use_obstacle_buffer")._value
        if use_obstacle_buffer:
            obs_buffer_size = self.get_parameter("obs_buffer_size").get_parameter_value().integer_value
            obs_buffer_period = self.get_parameter("obs_buffer_period").get_parameter_value().integer_value
            self.obstacle_buffer = CircularTimeBuffer(obs_buffer_size, obs_buffer_period, null_settable=self.buffer_accept_none)
        else:
            # set to False, as initiating any buffer (or deque) would result in True if self.x_buffer
            self.obstacle_buffer = -1
        # costmap buffer (default: Enabled)
        use_costmap_buffer = self.get_parameter("use_costmap_buffer")._value
        if use_costmap_buffer:
            cmap_buffer_size = self.get_parameter("map_buffer_size").get_parameter_value().integer_value
            cmap_buffer_period = self.get_parameter("map_buffer_period").get_parameter_value().integer_value
            self.costmap_buffer = CircularTimeBuffer(cmap_buffer_size, cmap_buffer_period, null_settable=self.buffer_accept_none)
        else:
            self.costmap_buffer = -1
        # Global obstacle buffer (default: Disabled) - obstacles in the global map
        use_global_buffer = self.get_parameter("use_global_buffer")._value
        if use_global_buffer:
            global_buffer_size = self.get_parameter("global_buffer_size").get_parameter_value().integer_value
            global_buffer_period = self.get_parameter("global_buffer_period").get_parameter_value().integer_value
            self.global_buffer = CircularTimeBuffer(global_buffer_size, global_buffer_period, null_settable=self.buffer_accept_none)
        else:
            self.global_buffer = -1

        """ setup tf related """ # TODO: analyze proper handling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dynamic_tf_pub = self.create_publisher(TransformStamped, 'dynamic_tf', 10)


        # This enables the ordering of rules. In case the order in which the rules are applied are important (shouldnt)
        self.sequence = self.get_parameter("sequence").get_parameter_value().integer_value # TODO: implement
        self.semantic_classification = self.get_parameter("semantic_classification")._value  # see _check_semantics

        # rule type decides if configuring is done via parameters or code: choices = ['parameters', 'code']
        # Use code if both direction and cost are custom, otherwise use parameters
        self.rule_type = self.get_parameter("rule_type")._value
        # TODO: misnomers?
        """
        direction type decides the direction of the costs: choices = ['front', 'all_sides', 'sector', 'gradient', 'custom']
            front -> costs are added to front of object. 
            all_sides -> costs are added to each side.
            sector -> costs are added to a sector infront of the object 
            gradient -> costs are added all around each side. # TODO: is this misnomer?
            TODO # custom -> implement own direction choice via "to_be_decided_function"
        """
        self.direction_type = self.get_parameter("direction_type")._value

        """
        cost type decides the way costs change: choices = ['falloff', 'reverse_falloff', 'custom']
        
        falloff decreases the cost each cell
            Used Params:
                base_cost - cost at origin (in the following examples base_cost = 10 is assumed)
                falloff_type - determines the way falloff is applied. See '_get_falloff_costs' for implementation
                    rel_percentage - percentage from last cell (e.g 10, 5, 2.5, 1.25 for 4 cells with 0.5 falloff)
                    abs_percentage - additive percentage from first cell (e.g 10, 9, 8, 7 for 0-3 cells with 0.1 falloff)
                    linear (unneeded?) - additive from last cell (e.g 10, 9, 8, 7 for 4 cells with 1 falloff) 
                    custom - override "get_custom_falloff_costs" to apply your own falloff type
                falloff (0-1 for *_percentage, x for linear) - determines the costs decrease each cell
                min_range - determines the distance from object to apply costs
                velocity_segments - determines the number of sub-vectors along the path. |v|=1 for distance_falloff
                reverse_falloff applies costs from max_range -> min_range instead of min_range->max_range. 
            Practically like setting falloff negative with regular falloff type. However with custom, it enables some different approaches
        
        custom applies your own function. To use, overwrite "get_custom_costs(self):"
        """
        self.cost_type = self.get_parameter("cost_type")._value

        self._check_params()
        self.height_filter = self.get_parameter("height_filter")._value
        # TODO: add lambda choice?
        # self.falloff_type = self.get_parameter("falloff_type")._value
        # self.falloff = self.get_parameter("falloff")._value
        # self.velocity_segments = self.get_parameter("velocity_segments")._value
        # self.min_range = self.get_parameter("min_range")._value
        # could we have a boolean to check for current path?
        # intersection_check = fields.Boolean

        # parameters for cell size?  # TODO: implement or remove
        self.cell_unit = "m"
        self.cell_size = 1
        self.convert_cell_size = False  # use real dimensions for cost calculation. useful for custom costmaps with high resolution

    """    CALLBACK    """
    def track_callback(self, msg):
        # debug
        # update delta time
        # TODO: sometimes dt is negative?? what to do with that? - skip = temporary fix
        dsec = msg.header.stamp.sec - self.sec
        dnsec = msg.header.stamp.nanosec - self.nanosec
        dt = dsec + dnsec / 1e9
        if dt < 0:  # temporary fix for negative dt
            if self.testing:
                self.get_logger().info(f"Skipping callback of old message")
            return
        self.sec = msg.header.stamp.sec
        self.nanosec = msg.header.stamp.nanosec

        # get obstacles
        self.obstacles = msg.obstacles
        # TODO: ensure transform
        # calculate costmap
        self.costmap = self.get_costmap(msg, dt)

        # update buffers
        self.update_buffers(dsec, dnsec)
        if self.testing:
            test_logging_data = [self.costmap.data]
            self.get_logger().info(f"Here is current data:{test_logging_data} \n\nEND DATA")
        # TODO: proper publsihing logic?
        if self.obstacles:
            if self.testing:
                self.get_logger().info(F"PUblisehd message!!!!!!!!!!!!!!!!!")
            self.costmap_publisher.publish(self.costmap)

    """    COSTMAP HANDLING    """

    """ CUSTOM COSTMAP HANDLER FUNCTIONS"""
    # noinspection PyMethodMayBeStatic
    def get_custom_costmap(self, msg, time_delta=False):
        """
        Return a OccupancyGrid object that can be used as a costmap layer.
        """
        return NotImplementedError("You know... "
                                   "You need to actually override 'get_custom_costmap' for 'code' rule type to work...")

    # noinspection PyMethodMayBeStatic
    def get_custom_costs(self):
        """
        Return a list of costs for each object detected.
        This list will be handled according to your direction_type.
        If you use builtin direction_types the costs are expected in the following format:
            # TODO: cost formatting
        """
        return NotImplementedError("You have to overwrite 'get_custom_costs' in order to use 'custom' cost type")

    def get_custom_costs_along_trajectory(self, start_point, velocity, costs, is_tuple=False):
        """
        this output dictionary needs to contain the following keys as well:
        border_attributes = ['min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z']
        """
        # you can also do this, if you only want to calculate the costs list in a custom way
        self.cost_type = 'velocity_falloff'
        data = self.calculate_costs_along_trajectory()
        self.cost_type = 'custom'
        return NotImplementedError("You have to overwrite 'get_custom_costs_along_trajectory' in order to use 'custom' cost type")

    # noinspection PyMethodMayBeStatic
    def get_custom_directions(self, prev_costmap=False, costs=False, time_delta=False):
        """
        # TODO: what is this :)
        """
        return NotImplementedError("You have to overwrite 'get_custom_directions' in order to use 'custom' directions type")

    # noinspection PyMethodMayBeStatic
    def get_custom_falloff_costs(self, obstacle_length):
        """
        Return a list of costs. This list will be handled according to your cost type (reversed for reverse_falloff).
        """
        return NotImplementedError("You have to overwrite 'get_custom_falloff_costs' in order to use 'custom' falloff type")

    """ BUILTIN COSTMAP HANDLER METHODS"""
    # GETTERS
    # Get the costmap based on rule
    def get_costmap(self, msg, time_delta=False):
        self._check_params()  # refreshes params and checks their validity - no need to check or set params again

        if self.rule_type == 'code':
            return self.get_custom_costmap(msg, time_delta)
        else:
            return self._get_direction_costmap(msg, time_delta)

    def _get_direction_costmap(self, msg, time_delta):
        """
        Creating a costmap involves the following steps:
        1) Localize the objects -> objects are found in self.obstacles
        2) calculate costs for each object based on their semantic data
            2) We update semantic data options here, in case new categorisation has been added by user
        3) apply the costs around the object.
            3.1) Optionally you can save the costs in 3D
        4) filter the costs by height, creating a basic costmap
        5) format the costmap into an OccupancyGrid
            5.1) transforms
            5.2) object composition
        @param time_delta: time passed from last callback in (s).
        For obstacles this time diff reflects buffer[0] -> self.obstacles
        @return: OccupancyGrid costmap
        """
        # TODO: i'm not sure how to handle previous costmaps...
        costmap = self.get_empty_costmap(msg)
        costmap.data = self.costmap.data
        if self.cost_type == 'custom':
            costs = self.get_custom_costs()

        else:
            costs = self._get_direction_costs()
        if self.direction_type == 'custom':
            vector_costs = self.get_custom_directions(costmap, costs, time_delta)
        else:
            vector_costs = self._get_directions(costmap, costs, time_delta)
        transformed_vector_costs = self.transform_dictionary_to_new_frame_and_publish(vector_costs, msg.header)
        self.get_logger().info(f"\n\n\n\n Before i implement, let's see: {transformed_vector_costs}\n\n "
                               f"This comes from origin: {[costmap.info.origin.position.x, costmap.info.origin.position.y, costmap.info.origin.position.z]}\n\n"
                               f"and from cmap frame {costmap.header.frame_id} and obstacle frame {msg.header.frame_id}\n\n"
                               f"OKay, this should be fine... \n\n")
        if len(transformed_vector_costs) == 0:
            if self.testing:
                self.get_logger().info(F"\nHmmm.. we have no data...\n")
            data = False
        else:
            data = self.fill_costmap(costmap, transformed_vector_costs)
        # TODO transform data
            costmap.data = data
        self.get_logger().info(F"\n\n\n Let's go: {data}\nbtw: {[len(self.obstacles), self.obstacles]}\n\n")
        return costmap

    def fill_costmap(self, costmap, vector_costs):
        cmap_data = costmap.data
        full_cmap_prob_dict = {}
        cmap_data_dict = {}
        show_first = True
        for object_id, trajectory in vector_costs.items():
            original_object = trajectory.pop('original_obstacle')
            for center_point, cost in trajectory.items():
                object_size = original_object.size
                cmap_prob_dict = self.map_object_to_costmap(center_point, object_size, costmap)
                full_cmap_prob_dict.update(cmap_prob_dict)
                if self.testing:
                    for key, value in cmap_prob_dict.items():
                        for fkey, fvalue in full_cmap_prob_dict.items():
                            if (key[0] == fkey[0] and key[1] == fkey[1]) or show_first:
                                self.get_logger().info(f"\n\n\n {not show_first} Found overlap!!! Full:{[fkey, fvalue]} -> current: {[key, value]}")
                                show_first = False
                cmap_data = self.apply_cost_to_objects(cmap_prob_dict, cost)
                cmap_data_dict.update(cmap_data)
        if self.testing:
            self.get_logger().info(F"\nPreparation complete... {cmap_data}\n\n")
        cmap_data = self.costmap_dict_as_list(cmap_data_dict, costmap)
        return cmap_data

    def _get_directions(self, costmap, costs, time_delta):
        border_attributes = ['min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z']
        size_attributes = ['min_size_x', 'max_size_x', 'min_size_y', 'max_size_y', 'min_size_z', 'max_size_z']
        sign = [1, -1]
        defaults = [self.width*self.resolution, self.height*self.resolution, self.height*self.width*(self.resolution**2)]
        direction_costs = {}
        # set default borders and sizes.
        [direction_costs.update({a: sign[i % 2]*defaults[int((i-1)/2)], size_attributes[i-1]: 0}) for a, i in zip(border_attributes, range(1, len(border_attributes) + 1))]

        if self.testing:
            self.get_logger().info(f" Created basis for map size: {direction_costs}")

        if self.direction_type == 'front':
            for obstacle in self.obstacles:
                trajectory_costs = self.calculate_costs_along_trajectory(obstacle.position, obstacle.velocity, costs, is_tuple=False)
                trajectory_costs = self.correct_trajectory_to_object_size(obstacle, trajectory_costs)
                # TODO: if 'sector' work, we can replace this with 'check_obstacle_attributes'...
                o_size = (obstacle.size.x, obstacle.size.y, obstacle.size.z)
                # check if borders or biggest object increase - also removes border attributes from trajectory costs
                for a, i in zip(border_attributes, range(1, len(border_attributes)+1)):
                    coordinate = trajectory_costs.pop(a)
                    size = o_size[int((i-1)/2)]  # gets size in the following order, [0, 0, 1, 1, 2, 2]
                    if i % 2: # min
                        if size < direction_costs[size_attributes[i-1]]:
                            direction_costs[size_attributes[i-1]] = size
                        if coordinate < direction_costs[a]:
                            direction_costs[a] = coordinate
                    else: # max
                        if size > direction_costs[size_attributes[i-1]]:
                            direction_costs[size_attributes[i-1]] = size
                        if coordinate > direction_costs[a]:
                            direction_costs[a] = coordinate
                # todo: ...and this with 'update_obstacle_dict'
                obstacle_uuid = uuid.UUID(bytes=bytes(obstacle.uuid.uuid))
                trajectory_costs.update({'original_obstacle': obstacle})
                direction_costs[obstacle_uuid] = trajectory_costs
            if self.testing:
                self.get_logger().info(f"\n\n\n  I have final directions: {direction_costs} from {len(self.obstacles)} obstacles: ")
            return direction_costs

        if self.direction_type == 'sides':
            # this is copy paste from 'front' except calculating the velocities for each side in a loop
            # TODO: all this needs is to calculate costs with a vector x = -y, y or -y and y = x, -x or -x for 90, 270, 180 around z? is around z correct?
            for obstacle in self.obstacles:
                trajectory_costs = {}
                right_vector = Vector3
                left_vector = Vector3
                right_vector.x, right_vector.y = obstacle.velocity.y * -1, obstacle.velocity.x
                left_vector.x, left_vector.y = obstacle.velocity.y, obstacle.velocity.x * -1
                sides = [obstacle.velocity, left_vector, right_vector]
                for side_velocity in sides:
                    side_costs = self.calculate_costs_along_trajectory(obstacle.position, side_velocity, costs, is_tuple=False)
                    side_costs = self.correct_trajectory_to_object_size(obstacle, side_costs)
                    trajectory_costs.update(side_costs)
                # TODO: does this work?
                self.check_obstacle_attributes(obstacle, border_attributes, size_attributes, trajectory_costs, direction_costs)
                direction_costs = self.update_obstacle_dict(direction_costs, obstacle, trajectory_costs, include_original_in_value_dict=True)

            if self.testing:
                self.get_logger().info(f"\n\n\n  I have final sector directions: {direction_costs} from {len(self.obstacles)} obstacles: ")
            return direction_costs
            # raise NotImplementedError("I'm not sure this functionality is useful. We may not implement this after all")
        if self.direction_type == 'sector':
            # TODO: NEW PARAMS
            #  sector-width param (degrees),
            #  fill policy - sides / sides + center / uniform / size based, if we start to differentiate object regions
            raise NotImplementedError("TO be implemented soon")
        if self.direction_type == 'gradient':
            raise NotImplementedError("TO be implemented soon")

    def _get_direction_costs(self):
        """
        returns a list of costs for a specific direction
        """
        # if cost_type in ['falloff', 'reverse_falloff']:
        if self.convert_cell_size:
            # if range in distance
            obstacle_length = self.velocity_segments/self.cell_size  # 9-2/ 2 = 3.5 -> 0, 0, 10, 10,
            raise NotImplementedError("Using cell_size and distance-based ranges are not yet implemented. \n"
                                       "Please set parameter: 'convert_cell_size: False or use 'custom' cost_type")
        else:
            # if range in costmap cells
            # 9 - 2 = 7 -> 0c0, 0c1, 1c2, 2c3, 3c4, 4c5, 5c6, 6c7, 7c8, 8c9,
            obstacle_length = self.velocity_segments  # since both ranges are inclusive, we need to add 1:
            costs = self._get_falloff_costs(obstacle_length)
            if len(costs) != obstacle_length:
                raise ValueError("Received different amount of falloff than specified by range.")
            if self.reverse_falloff:
                costs.reverse()
            # self.check_cost_length(costs)
            return costs

    def _get_falloff_costs(self, obstacle_length):
        """
        Calculate the costs for an object moving at an unit vector
        @param obstacle_length:
        @return: list of costs with length obstacle_length (ideally 1 / resolution)
        """
        costs = []
        if self.falloff_type == 'rel_percentage':
            costs = [self.base_cost * (self.falloff ** distance) for distance in range(obstacle_length)]
        if self.falloff_type == 'abs_percentage':  # TODO: abs_percentage is the same as linear...
            costs = [self.base_cost * (1 - distance * self.falloff) for distance in range(obstacle_length)]
        if self.falloff_type == 'linear':
            costs = [self.base_cost - distance * self.falloff for distance in range(obstacle_length)]

        if self.falloff_type == 'custom':
            costs = self.get_custom_falloff_costs(obstacle_length)
        return costs

    """    BUFFER HANDLING    """
    # By default we use only seconds for period,
    # but for those that desire utmost precision, checking period by ns is available
    def update_buffers(self, delta_time, delta_ns=0, buffer='all'):
        if self.obstacle_buffer != -1 and buffer in ['all', 'local', 'obstacle']:
            obs = self.update_obs_buffer(delta_time, delta_ns)
            if self.testing and obs:
                self.get_logger().info(f"Updated obstacles buffer: {[self.obstacle_buffer], delta_time, delta_ns}")
        if self.costmap_buffer != -1 and buffer in ['all', 'local', 'costmap']:
            cmap = self.update_costmap_buffer(delta_time, delta_ns)
            if self.testing and cmap:
                self.get_logger().info(f"Updated costmap buffer: {self.costmap_buffer}")
        if self.global_buffer != -1 and buffer in ['all', 'global']:
            gmap = self.update_global_buffer(delta_time, delta_ns)
            if self.testing and gmap:
                self.get_logger().info(f"Updated costmap buffer: {self.costmap_buffer}")

    def update_obs_buffer(self, delta_time, delta_ns=0):
        return self.obstacle_buffer.appendleft(self.obstacles, delta_time, delta_ns)

    def update_costmap_buffer(self, delta_time, delta_ns=0):
        return self.costmap_buffer.appendleft(self.costmap, delta_time, delta_ns)

    def update_global_buffer(self, delta_time, delta_ns):
        return self.global_buffer.appendleft(self.global_map, delta_time, delta_ns)

    """ Information handling and generators """
    def get_empty_costmap(self, msg):
        costmap = OccupancyGrid()
        costmap.header = msg.header  # TODO:
        costmap.info = self.get_metadata(msg)
        if not costmap.header.frame_id:
            costmap.header.frame_id = self.get_parameter("costmap_frame").get_parameter_value().string_value
        # py_empty_map = PyCostmap2D(costmap)
        return costmap

    def get_metadata(self, msg):
        data = MapMetaData()
        # if MapMetaData in msg we could use that, but in our case there isn't so we use params
        data.resolution = self.get_parameter("resolution").get_parameter_value().double_value
        data.width = self.get_parameter("width").get_parameter_value().integer_value
        data.height = self.get_parameter("height").get_parameter_value().integer_value
        # data.map_load_time = self.get_parameter("map_load_time")._value
        # data.origin = None
        return data

    @staticmethod
    def normalize_cmap_coordinates(xyz, is_tuple=False):
        if is_tuple:
            return int(xyz[0]), int(xyz[1]), int(xyz[2])
        return int(xyz.x), int(xyz.y), int(xyz.z)

    @staticmethod
    def position_to_indices(position, resolution, origin, normalize=True):
        # resolution = cmap.info.resolution
        # origin = cmap.info.origin
        if isinstance(origin, tuple):
            translation = Point()
            translation.x, translation.y, translation.z = origin
        else:
            translation = origin.position

        if isinstance(position, tuple):
            x_idx = (position[0] - translation.x) / resolution
            y_idx = (position[1] - translation.y) / resolution
            z_idx = (position[2] - translation.z) / resolution
        else:
            x_idx = (position.x - translation.x) / resolution
            y_idx = (position.y - translation.y) / resolution
            z_idx = (position.z - translation.z) / resolution
        if normalize:
            return int(x_idx), int(y_idx), int(z_idx)
        return x_idx, y_idx, z_idx

    @staticmethod
    def size_to_indices(size, resolution, normalize=True):
        """
        converts 3D coordinates into another system based on resolution
        @param size: Point object with attributes (x, y, z)
        @param resolution: give from cmap PyCostmap2D.getResolution() or OccupancyGrid.info.resolution
        @param normalize: whether to normalize the coordinates into their integer values
        @return: 3D coordinates: x, y, z
        """
        # Assuming size is in meters, convert to cells
        x_idx = size.x / resolution
        y_idx = size.y / resolution
        z_idx = size.z / resolution
        if normalize:
            return int(x_idx), int(y_idx), int(z_idx)
        return x_idx, y_idx, z_idx

    @staticmethod
    def tuple_to_indices(size, resolution, normalize=True):
        """
        converts 3D coordinates into another system based on resolution
        @param size: 3D coordinates in a tuple or list: (x, y, z)
        @param resolution: give from cmap PyCostmap2D.getResolution() or OccupancyGrid.info.resolution
        @param normalize: whether to normalize the coordinates into their integer values
        @return: 3D coordinates: x, y, z
        """
        # Assuming size is in meters, convert to cells
        x_idx = size[0] / resolution
        y_idx = size[1] / resolution
        z_idx = size[2] / resolution
        if normalize:
            return int(x_idx), int(y_idx), int(z_idx)
        return x_idx, y_idx, z_idx

    @staticmethod
    def normalize_vector(velocity, is_tuple=False):
        if is_tuple:
            magnitude = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2)
            normal_vector = velocity[0] / magnitude, velocity[1] / magnitude, velocity[2] / magnitude
            return normal_vector, magnitude
        magnitude = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        normal_vector = Vector3()
        normal_vector.x = velocity.x / magnitude
        normal_vector.y = velocity.y / magnitude
        normal_vector.z = velocity.z / magnitude
        return normal_vector, magnitude

    @staticmethod
    def costs_to_vector(direction, costs, is_tuple=False):
        if is_tuple:
            direction_tuple = direction
        else:
            direction_tuple = (direction.x, direction.y, direction.z)
        cost_dict = {}
        for i in range(len(costs)):
            cost_vector = (direction_tuple[0] * i / len(costs), direction_tuple[1] * i / len(costs), direction_tuple[2] * i / len(costs))
            cost_dict[cost_vector] = costs[i]
        return cost_dict

    def calculate_costs_along_trajectory(self, start_point, velocity, costs, is_tuple=False):
        # Calculate step size for each point
        step = 1 # (velocity[0] / num_points, velocity[1] / num_points, velocity[2] / num_points)
        if self.cost_type == 'custom':
            return self.get_custom_costs_along_trajectory(start_point, velocity, costs, is_tuple)
        # Initialize list to store costs
        costs_along_trajectory = {}
        direction, magnitude = self.normalize_vector(velocity)
        magnitude = magnitude * self.velocity_duration
        # vector_costs = self.costs_to_vector(velocity, costs, is_tuple=is_tuple)  # maps cost to vector not point
        # Calculate cost for each point along the trajectory
        current_point = Point()
        origin_point = Point()
        if is_tuple:
            current_point.x, current_point.y, current_point.z = start_point
            origin_point.x, origin_point.y, origin_point.z = start_point
        else:
            current_point.x, current_point.y, current_point.z = start_point.x, start_point.y, start_point.z
            origin_point.x, origin_point.y, origin_point.z = start_point.x, start_point.y, start_point.z
        if self.testing:
            self.get_logger().info(F"My logic tracks: {len(costs)} == {self.velocity_segments}")
        for i in range(len(costs)):
            # Calculate magnitude of velocity vector at this point
            if self.cost_type == 'velocity_falloff':
                current_point.x = origin_point.x + direction.x*magnitude*i/len(costs) + direction.x * self.min_range / self.resolution
                current_point.y = origin_point.y + direction.y*magnitude*i/len(costs) + direction.y * self.min_range / self.resolution
                current_point.z = origin_point.z + direction.z*magnitude*i/len(costs) + direction.z * self.min_range / self.resolution
            elif self.cost_type == 'distance_falloff':
                current_point.x = origin_point.x + direction.x*i + direction.x * self.min_range / self.resolution
                current_point.y = origin_point.y + direction.y*i + direction.y * self.min_range / self.resolution
                current_point.z = origin_point.z + direction.z*i + direction.z * self.min_range / self.resolution
            # Append cost and current point to the dictionary
            # we cannot use Point as key so we transfer to tuple after all. But i think it's better to use Point in gen.
            key_point = current_point.x, current_point.y, current_point.z
            costs_along_trajectory[key_point] = costs[i]
            self.get_logger().info(f"currentpoint:  {[current_point.x, current_point.y, current_point.z]} and origin {[origin_point.x, origin_point.y, origin_point.z]}")

        # map the furthest point:
        if direction.x > 0:
            min_x = origin_point.x
            max_x = current_point.x
        else:
            min_x = current_point.x
            max_x = origin_point.x
        if direction.y > 0:
            min_y = origin_point.y
            max_y = current_point.y
        else:
            min_y = current_point.y
            max_y = origin_point.y
        if direction.z > 0:
            min_z = origin_point.z
            max_z = current_point.z
        else:
            min_z = current_point.z
            max_z = origin_point.z
        border_attributes = {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y, 'min_z': min_z, 'max_z': max_z}
        self.get_logger().info(f"\n\nCosts along trajectory: {costs_along_trajectory}\n So we map {border_attributes} \n because current {[current_point.x, current_point.y, current_point.z]} and origin {[origin_point.x, origin_point.y, origin_point.z]} and start {[start_point.x, start_point.y, start_point.z]} or maybe {key_point}")
        costs_along_trajectory.update(border_attributes)
        return costs_along_trajectory

    def correct_trajectory_to_object_size(self, obstacle, trajectory_costs):
        self.get_logger().info(f"We still haven't corrected trajectories for obstacle size....")
        return trajectory_costs

    @staticmethod
    def calculate_overlap_probability_2d(grid_cell, object_center, object_size):
        object_width = object_size[0]
        object_length = object_size[1]
        cell_x, cell_y = grid_cell

        cell_max_x = cell_x + 1
        cell_max_y = cell_y + 1

        object_min_x = object_center[0] - object_width / 2
        object_max_x = object_center[0] + object_width / 2
        object_min_y = object_center[1] - object_length / 2
        object_max_y = object_center[1] + object_length / 2

        overlap_x = max(0, min(cell_max_x, object_max_x) - max(cell_x, object_min_x))
        overlap_y = max(0, min(cell_max_y, object_max_y) - max(cell_y, object_min_y))

        overlap_area = overlap_x * overlap_y
        cell_area = 1  # Assuming each cell is 1x1

        return overlap_area / cell_area

    def map_object_to_costmap(self, object_center, object_size, costmap):
        origin = costmap.info.origin
        resolution = self.resolution or costmap.info.resolution  # TODO: which to prefer?
        cmap_center = self.position_to_indices(object_center, resolution, origin, normalize=False)
        normal_cmap_center = self.normalize_cmap_coordinates(cmap_center, is_tuple=True)
        cmap_size = self.size_to_indices(object_size, resolution, normalize=False)
        cmap_size_x, cmap_size_y, cmap_size_z = cmap_size
        cmap_probability_dict = {}
        for x in np.arange(normal_cmap_center[0]-(int(cmap_size_x/2)+1), normal_cmap_center[0]+int(cmap_size_x/2)+1, 1):
            for y in np.arange(normal_cmap_center[1] - (int(cmap_size_y / 2) + 1),
                               normal_cmap_center[1] + int(cmap_size_y / 2) + 1, 1):
                position = (x, y)
                prob = self.calculate_overlap_probability_2d(position, cmap_center, cmap_size)
                cmap_probability_dict[position] = int(round(prob*100, 8))
        return cmap_probability_dict

    @staticmethod
    def apply_cost_to_objects(probability_dict, cost):
        # probability dict uses integer probability to be suitable for direct list conversion
        cost_dict = {}  # probability_dict
        for key, value in probability_dict.items():
            cost_dict[key] = int(value/100*cost)  # TODO what is the proper cost?
        return cost_dict

    def costmap_dict_as_list(self, costmap_dict, costmap):
        width = costmap.info.width
        height = costmap.info.height
        cost_list = [-1] * width*height  # costmap.data
        # py_costmap = PyCostmap2D(costmap)
        for key, value in costmap_dict.items():
        # for (x, y), value in costmap_dict.items():
            if not isinstance(key, tuple):
                warnings.warn(f"Detected non tuple key in costmap dictionary. Is this intentional? Key in question: {key}")
                continue
            if len(key) > 2 and not self.valid_height(key): # or self.valid_cost(value)
                continue
            # py_costmap.setCost(key[0], key[1], value)
            cost_list[key[0]+key[1]*width] = value
        # data = py_costmap.costmap
        if self.testing:
            self.get_logger().info(f"\n\ndata for costmap: {[len(cost_list), type(cost_list)]}\n\n direct Costmap: {cost_list}\n\n")
        return cost_list

    """ Transform functions """
    def transform_dictionary_to_new_frame_and_publish(self, input_dict, input_header, new_frame_id='semantic_rule_dynamic_transform'):
        # Find the minimum x, y, and z values
        min_x = input_dict.get('min_x', 0)
        min_y = input_dict.get('min_y', 0)
        min_z = input_dict.get('min_z', 0)
        min_size_x = input_dict.get('min_size_x', 0)
        min_size_y = input_dict.get('min_size_y', 0)
        min_size_z = input_dict.get('min_size_z', 0)
        max_size_x = input_dict.get('max_size_x', 0)
        max_size_y = input_dict.get('max_size_y', 0)
        max_size_z = input_dict.get('max_size_z', 0)
        rot_x = input_dict.get('rot_x', 0.0)
        rot_y = input_dict.get('rot_y', 0.0)
        rot_z = input_dict.get('rot_z', 0.0)
        rot_w = input_dict.get('rot_w', 1.0)
        self.get_logger().info(f"\n\n\n\n Hello: {input_dict}\n\n\n")

        # Create a new transform message
        transform_msg = TransformStamped()
        transform_msg.header.frame_id = new_frame_id
        transform_msg.child_frame_id = input_header.frame_id  # Assuming header exists
        transform_msg.header.stamp = input_header.stamp # self.get_clock().now().to_msg()
        # todo: should we translate positive values?
        min_x = 0 if min_x > 0 else min_x
        min_y = 0 if min_y > 0 else min_y
        min_z = 0 if min_z > 0 else min_z
        # Set the translation
        translation_x = transform_msg.transform.translation.x = -1*(min_x - max(abs(min_size_x), abs(max_size_x)))
        translation_y = transform_msg.transform.translation.y = -1*(min_y - max(abs(min_size_y), abs(max_size_y)))
        translation_z = transform_msg.transform.translation.z = -1*(min_z - max(abs(min_size_z), abs(max_size_z)))

        # Set the rotation (identity quaternion)
        transform_msg.transform.rotation.x = rot_x
        transform_msg.transform.rotation.y = rot_y
        transform_msg.transform.rotation.z = rot_z
        transform_msg.transform.rotation.w = rot_w

        # Publish the transform
        self.dynamic_tf_pub.publish(transform_msg)

        # Apply the translation to each coordinate tuple
        transformed_dict = {}
        for key, value in input_dict.items():
            if isinstance(value, dict):
                transformed_coords = {}
                for coord_tuple, cost in value.items():
                    if not isinstance(coord_tuple, str):
                        transformed_coord = (
                            coord_tuple[0] + translation_x,
                            coord_tuple[1] + translation_y,
                            coord_tuple[2] + translation_z
                        )
                        transformed_coords[transformed_coord] = cost
                    elif coord_tuple == 'original_obstacle':
                        transformed_coords['original_obstacle'] = cost  # actually this is object for filling costmap
                transformed_dict[key] = transformed_coords
        return transformed_dict

    """  Validation functions """
    def valid_height(self, coordinates):
        if isinstance(coordinates, tuple):
            return self.height_filter[0] <= coordinates[2] <= self.height_filter[1]
        else:
            return self.height_filter[0] <= coordinates.z <= self.height_filter[1]

    # def check_cost_length(self, costs, costmap):
    #     len = len(costs)
    @staticmethod
    def check_obstacle_attributes(obstacle, border_attributes, size_attributes, trajectory_costs, direction_costs):
        o_size = (obstacle.size.x, obstacle.size.y, obstacle.size.z)
        # check if borders or biggest object increase - also removes border attributes from trajectory costs
        for a, i in zip(border_attributes, range(1, len(border_attributes) + 1)):
            coordinate = trajectory_costs.pop(a)
            size = o_size[int((i - 1) / 2)]  # gets size in the following order, [0, 0, 1, 1, 2, 2]
            if i % 2:  # min
                if size < direction_costs[size_attributes[i - 1]]:
                    direction_costs[size_attributes[i - 1]] = size
                if coordinate < direction_costs[a]:
                    direction_costs[a] = coordinate
            else:  # max
                if size > direction_costs[size_attributes[i - 1]]:
                    direction_costs[size_attributes[i - 1]] = size
                if coordinate > direction_costs[a]:
                    direction_costs[a] = coordinate

    @staticmethod
    def update_obstacle_dict(dictionary, obstacle, value, include_original_in_value_dict=False):
        """
        Update a dictionary where the keys are
        @param dictionary: Dictionary to update, where keys are obstacle uuid-s
        @param obstacle: the key to be updatedoriginal
        @param value: value at key
        @include_original_in_value_dict boolean: if value is a dict, it may be useful to also pass the full object there
        @return: dictionary with the updated value
        """
        obstacle_uuid = uuid.UUID(bytes=bytes(obstacle.uuid.uuid))
        if include_original_in_value_dict:
            value.update({'original_obstacle': obstacle})
        dictionary[obstacle_uuid] = value
        return dictionary


def main(args=None):
    rclpy.init(args=args)
    rule_node = SemanticCostmapRule()
    rclpy.spin(rule_node)
    rule_node.destroy_node()
    rclpy.shutdown()