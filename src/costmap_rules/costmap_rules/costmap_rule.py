# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs import Pose
from visualization_msgs.msg import Marker, MarkerArray
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray  # TODO: ensure this is properly imported

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class CostmapRule(Node):

    def __init__(self):
        super().__init__('detection_costmap_rule')
        # TODO: evaluate whether or not to save parameters as class variables
        self.declare_parameters(
            namespace='',
            parameters=[
                # input params
                ('detection_topic', "dynamic_object"),  # remapping is better then using parameters?
                ('sequence', 1),  # This can be used to set the order of layer creation
                # cost calculation params
                ('direction_type', "linear"),
                ('cost_type', "falloff"),
                ('falloff_type', "rel_percentage"),
                ('base_cost', 10),
                ('falloff', 0.5),
                ('min_range', 0),
                ('max_range', 10),
                # custom params?
                ('global_frame', "camera_link"),
                ('process_noise_cov', [2., 2., 0.5]),
                ('top_down', False),
                ('death_threshold', 3),
                ('measurement_noise_cov', [1., 1., 1.]),
                ('error_cov_post', [1., 1., 1., 10., 10., 10.]),
                ('vel_filter', [0.1, 2.0]),
                ('height_filter', [-2.0, 2.0]),
                ('cost_filter', 1.0)
            ])
        self.detection_topic = self.get_parameter("detection_topic")._value
        # TODO: make topic_name dynamic based on param (detection_publisher -> self.detection_topic)
        #  although nav2 cpp costmap takes the topicname as class init input...?
        self.costmap_publisher = self.create_publisher(OccupancyGrid, 'detection_publisher', 10)

        # subscribe to trackers from navigation2_dynamic
        self.tracker_sub = self.create_subscription(
            ObstacleArray,
            'tracking',
            self.track_callback,
            10)
        self.marker_sub = self.create_subscription(
            MarkerArray,
            'tracking_marker',
            self.marker_callback,
            10)

        self.detection_list = []
        self.sec = 0
        self.nanosec = 0

        # # publisher for detections - these should stay in navigation2_dynamic
        # self.tracker_obstacle_pub = self.create_publisher(ObstacleArray, 'detection_costmap', 10)
        # self.tracker_marker_pub = self.create_publisher(MarkerArray, 'tracking_marker', 10)

        # setup tf related # TODO: is this needed? what is this?
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # This enables the ordering of rules. In case the order in which the rules are applied is important
        self.sequence = self.get_parameter("sequence")._value 
        
        # rule type decides if configuring is done via parameters or code: choices = ['parameters', 'code']
        # You can use code based direction or costs separately too, by setting "custom"
        # Use code if both direction and cost are custom, otherwise use parameters
        self.rule_type = self.get_parameter("rule_type")._value
        # TODO: misnomers?
        """
        direction type decides the direction of the costs: choices = ['vector', 'linear', 'gradient', 'custom']
            vector -> costs are added to one side. 
            linear -> costs are added to each side.
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
                min/max_range determines the start and end of falloff (inclusive) (before min falloff = 0 and after max falloff = 1)
                
        reverse_falloff applies costs from max_range -> min_range instead of min_range->max_range. 
            Practically like setting falloff negative with regular falloff type. However with custom, it enables some different approaches
        
        custom applies your own function. To use, overwrite "get_custom_costs(self, min_range, max_range, cell_size):"
        """
        self.cost_type = self.get_parameter("cost_type")._value
        # TODO: implement
        # TODO: add lambda choice?
        # self.falloff_type = self.get_parameter("falloff_type")._value
        # self.falloff = self.get_parameter("falloff")._value
        # self.max_range = self.get_parameter("max_range")._value
        # self.min_range = self.get_parameter("min_range")._value
        # could we have a boolean to check for current path?
        # intersection_check = fields.Boolean

        # parameters for cell size?
        self.cell_unit = "m"
        self.cell_size = 1
        self.use_cell_size = False

    """
    CALLBACKS
    """
    def track_callback(self, msg):
        dt = self.update_delta_time(msg.header.stamp)

        if self.detect_changes():
            self.costmap_callback(msg)
        return
    # subsciber_callback(self, msg) - called when detection published - this should detect changes, and if existing
    # it should call:
    # publisher_callback(self, input=msg), which then uses the input to create the costmap
    def costmap_callback(self, msg):
        """
        The core function, that creates the costmap via "get_costmap" and publishes it
        # TODO: Decide whether this should be published on diff or timer
        :return: OccupancyGrid - a finalized costmap layer
        """
        res = OccupancyGrid()
        costmap = self.get_costmap(msg)  # TODO: in progress
        res.data = costmap  # TODO: format - # The map data, in row-major order, starting with (0,0).
        res.info = self.get_meta_data(msg)  # TODO: implement # type MapMetaData
        res.header = msg.header  # self.getHeader()  # TODO: implement # type Header
        # Occupancy probabilities are in the range [0,100].  Unknown is -1.
        self.publisher_.publish(res)
        # self.get_logger().info('Publishing: "%s"' % res.data)
        # return res

    """
    FORMATING and INFORMATIONAL FUNCTIONS
    """
    def get_meta_data(self):
        # TODO: implement
        return MapMetaData()

    """
    # should we allow smth like this? Maybe with lambda
    def clean_string(self, string):
        clean_string = string.strip()
        clean_string = clean_string.replace(_variable_start_character, '')
        clean_string = clean_string.replace(_variable_end_character, '')
        return clean_string

    def eval_string(self, locals_dict, string=False):
        check_string = False
        if string:
            check_string = string
        elif self.formula:
            check_string = self.formula
    
        if check_string:
            try:
                clean_formula = self.clean_string(check_string)
                value = safe_eval(clean_formula, locals_dict=locals_dict, nocopy=True)
            except Exception:
                value = False
        else:
            value = False
    
        return value
    """

    """
    GETTERS
    """
    def get_input(self):
        rule_type_choices = ['parameters', 'code']
        direction_type = ['vector', 'linear', 'gradient', 'custom']

        # Separate functions?
        rule_type = self.get_parameter("rule_type")._value
        direction_type = self.get_parameter("direction_type")._value
        cost_type = self.get_parameter("cost_type")._value
        max_range = self.get_parameter("max_range")._value
        min_range = self.get_parameter("min_range")._value

        if rule_type not in rule_type_choices:
            raise IOError(f"Selected rule type not implemented. Possible choices are {rule_type_choices}")
        if direction_type not in direction_type:
            raise IOError(f"Selected direction type not implemented. Possible choices are {direction_type}")
        # TODO: validate cost_type, max_range and min_range
        return rule_type, direction_type, cost_type, max_range, min_range
    """
    SETTERS
    """
    def update_delta_time(self, time_stamp):
        """

        :param time_stamp: stamp, usually from input message header
        :return: float - delta time (seconds), precision - nanosec ( 10^-9 )
        """
        dt = (time_stamp.sec - self.sec) + (time_stamp.nanosec - self.nanosec) / 1e9
        self.sec = time_stamp.sec
        self.nanosec = time_stamp.nanosec
        return dt
    """
    Evaluation / Math
    """
    # Subscriber (detection comparison)
    def detect_changes(self):
        return NotImplementedError("I HAVE SO MUCH TODO")


    # Publisher (costmap generation)
    def get_custom_falloff_costs(self, obstacle_length, base, falloff):
        """
        Return a list of costs. This list will be handled according to your cost type (reversed for reverse_falloff).
        """
        return NotImplementedError("You have to overwrite get_custom_falloff_costs in order to use 'custom' falloff type")

    def _get_falloff_costs(self, obstacle_length, base, falloff, falloff_type):
        falloff_choices = ['rel_percentage', 'abs_percentage', 'linear', 'custom']
        if falloff_type not in falloff_choices:
            raise IOError(f"Selected falloff type not implemented. Possible choices are {falloff_choices}")
        costs = []
        if falloff_type == 'rel_percentage':
            costs = [base*(falloff**distance) for distance in range(obstacle_length)]
        elif falloff_type == 'abs_percentage':
            costs = [base*(1-distance*falloff) for distance in range(obstacle_length)]
        elif falloff_type == 'linear':
            costs = [base + distance*falloff for distance in range(obstacle_length)]
        elif falloff_type == 'custom':
            costs = self.get_custom_falloff_costs(obstacle_length, base, falloff)
        else:
            pass
        return costs

    def get_custom_costs(self, min_range, max_range, cell_size=False, falloff="Not input?"):
        """
        Return a list of costs. This list will be handled according to your cost type (reversed for reverse_falloff).
        """
        return NotImplementedError("You have to overwrite get_custom_costs in order to use 'custom' cost type")

    def _get_direction_costs(self, cost_type, min_range, max_range, cell_size, falloff="Not input?"):
        """
        returns a list of costs for a specific direction
        """
        choices = ['falloff', 'reverse_falloff', 'custom']
        if cost_type not in choices:
            raise IOError(f"Selected cost type not implemented. Possible choices are {choices}")
        if cost_type in ['falloff', 'reverse_falloff']:
            if self.use_cell_size:
                # if range in distance
                obstacle_length = (max_range-min_range)/cell_size  # 9-2/ 2 = 3.5 -> 0, 0, 10, 10,
                return NotImplementedError("Using cell_size and distance-based ranges are not yet implemented. \n"
                                           "Please set parameter: 'use_cell_size: False")
            else:
                # if range in costmap cells
                # 9 - 2 = 7 -> 0c0, 0c1, 1c2, 2c3, 3c4, 4c5, 5c6, 6c7, 7c8, 8c9,
                obstacle_length = (max_range-min_range)+1  # since both ranges are inclusive, we need to add 1:
                map_length = 2*max_range-1
                base_cost = self.get_parameter("base_cost")._value
                falloff = self.get_parameter("falloff")._value
                falloff_type = self.get_parameter("falloff_type")._value
                costs = self._get_falloff_costs(obstacle_length, base_cost, falloff, falloff_type)
                if len(costs) != obstacle_length:
                    raise ValueError("Received different amount of falloff than specified by range.")
                if falloff_type == 'reverse_falloff':
                    return costs.reverse()
                else:
                    return costs
        if cost_type == 'custom':
            return self.get_custom_costs(min_range, max_range, cell_size)

    def get_custom_costmap(self):
        return NotImplementedError("You have to overwrite get_custom_costmap in order to use 'code' rule type")

    def get_costmap(self, msg="Not needed?"):
        """
        The costmap generation should follow these steps:
            1) Validation - check input - Done?
            2) Get current and previous state - # TODO: how to optimize for using only diff
                2.1) Own location (robot)
                2.2) Detections
                2.3) Markers? - i'm not yet sure what these entail anyway
            3) Map detections unto costmap
            4) Calculate rules:
                4.0) What times do we calculate? Having access to vectors we can make predictions for unlimited time.
                    4.0.1) IF we have access to expired data, we can calcualte probability of direction change
                4.1) directiony type decides how cost is implemented:
                    4.1.1) How we account for movement -
                        * Do we account for movement, or just make different costs for dynamic objects
                        * If and how do we account for our own movement?
                        * Do we remove moving obstacles that aren't on our path
                        * How do account for our own movement for probabilistic detections
                4.2) Calculate costs for each detection (mapped on ID)
                    * This means creating a mini-map with max_range radius (simplify to square max_range*2-1) around each object
                4.3) Create the costmap with dynamic objects should be indicated both probability and ID.
            5) apply costs with probability weight
                * do we use only the highest probability or use a weighted average or other method?  - INPUT
                * Do we Sum or max overlapping predictions - INPUT? customizeable
                * NB! Do not sum same ID costs
            6) Format output
        """
        # 1) Get and validate input
        rule_type, direction_type, cost_type, max_range, min_range = self.get_input()
        if rule_type == 'code_no_input':
            return self.get_custom_costmap_raw()

        # 2) Get current state
        costmap = self.get_current_costmap()  # TODO: implement
        # free_space = self.get_free_space()  # TODO: I believe this is needed for Detection -> costmap conversion...idk
        obstacles, markers = self.get_current_detections()  # TODO: implement
        current_position = self.get_current_position()  # TODO: implement

        # 3) # TODO: Map detections unto costmap

        # 4) # TODO: IN PROGREESS: calculate rules
        if rule_type == 'code':
            # should the user ask for input?
            return self.get_custom_costmap(costmap, obstacles, markers, current_position)
        # if rule_type == 'parameters':

        directions = self.get_directions()
        direction_costs = self._get_direction_costs(cost_type, min_range, max_range, self.cell_size)

