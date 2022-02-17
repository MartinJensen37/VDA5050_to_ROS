########################### IMPORTED MODULES AND MESSAGES ###########################################
# Standard or misc. python modules
from dataclasses import dataclass, field
import dataclasses
import sys
import datetime
import copy
from dotmap import DotMap
from typing import List, Dict

# JSON specific modules
import json
import jsonschema

# MQTT paho specific modules
import paho.mqtt.client as mqtt

# ROS specific modules
import rospy

# ROS specific messages
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal, MoveBaseActionFeedback


########################## GLOBAL VARIABLES AND CONSTANTS ###########################################

# These will be updated at major/minor changes e.g. x.x.0 when the x's are changing
ORDER_TOPIC = 'IS/v0.1.0/polybot/05/order'
INSTANT_ACTION = 'IS/v0.1.0/polybot/05/instantAction'
STATE_TOPIC = 'IS/v0.1.0/polybot/05/state'
CONNECTION_TOPIC = 'IS/v0.1.0/polybot/05/connection'

# Schema consts
ORDER_SCHEMA = 'order.schema'
INSTANT_ACTION_SCHEMA = 'instantAction.schema'
STATE_SCHEMA = 'state.schema'

# Operating modes
AUTOMATIC_OP_MODE = 'AUTOMATIC'
SEMIAUTOMATIC_OP_MODE = "SEMIAUTOMATIC"
MANUAL_OP_MODE = "MANUAL"
SERVICE_OP_MODE = "SERVICE"
TEACHIN_OP_MODE = "TEACHIN"

# Action status 
ACTION_WAITING = 'WAITING'
ACTION_INITIALIZING = 'INITIALIZING'
ACTION_RUNNING = 'RUNNING'
ACTION_PAUSED = 'PAUSED'
ACTION_FINISHED = 'FINISHED'
ACTION_FAILED = 'FAILED'

########################## CLASSES AND FUNCTION DEFINITIONS #########################################
@dataclass
class nodeState():
    node_id: str = None
    sequence_id: str = None
    released: bool = None
    node_pos: Dict[float, str] = field(default_factory = lambda: {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'allowed_div_xy': 0.0, 'allowed_div_theta': 0.0, 'map_id': '', 'map_description': ''})

@dataclass
class edgeState():
    edge_id: str = None
    sequence_id: str = None
    released: bool = None

@dataclass
class actionState():
    action_id: str = None
    action_type: str = None
    action_status: str = None # waiting, initializing, running, finished and failed

@dataclass
class batteryState():
    battery_charge: float = None
    charging: bool = None

@dataclass
class state():
    header_id: int = None
    order_id: str = None
    order_update_id: int = None
    timestamp: str = None
    version: str = None
    serial_number:str = None
    manufacturer: str = None
    last_node_id: str = None
    last_node_sequence_id = None
    driving: bool = True
    operating_mode: str = None # (Enum) only certain strings are accepted.
    agv_position: Dict[float, str] = field(default_factory = lambda: {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'map_id': '', 'map_description': '', "positionInitialized": True, "localizationScore":0.0, "deviationRange":0.0})
    node_states: List[nodeState] = None
    edge_states: List[edgeState] = None
    action_states: List[actionState] = None
    battery_states: List[batteryState] = None


class taskManager():

    def __init__(self, broker_addr = '192.168.1.127', port = 1883) -> None:

        rospy.init_node("taskManagerNode")

        # initialize rospy subscribers and publishers

        # publishers
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=0)
        #self.goal_pub = rospy.Publisher('/')

        # initialize and establish connection to MQTT broker 
        self.mqtt_client = mqtt.Client('Task Manager') #create new instance of the mqtt node
        self.mqtt_client.on_message = self.mqtt_cb # define default callback function
        result = self.mqtt_client.connect(broker_addr, port) #connect to broker
        self.mqtt_client.subscribe([(ORDER_TOPIC, 0), (INSTANT_ACTION, 0)])
        self.mqtt_client.loop_start()

        # create initial states for orderId, orderUpdateId, order_active
        self.order_prev_id = ''
        self.order_update_id = 0
        self.order_active = True # boolean
        self.header_id = 0 # This value is incremented every time a message is sent but is reset when the program is restarted
        self.action_status = ACTION_WAITING

        # create the classes for storing all robot state information
        self.state = state()
        self.state.last_node_id = 'warehouse'
        
    def robot_odom(self, data):
        self.state.agv_position['x'] = data.pose.pose.position.x
        self.state.agv_position['y'] = data.pose.pose.position.y
        self.state.agv_position['theta'] = data.pose.pose.orientation.z
        
    def robot_feedback(self, data):
        pass

    def robot_status(self, data):
        if data.status_list[0].status == 1:
            print("Robot is ACTIVE! Current GOAL id is: ", data.status_list[0].goal_id.id)
            self.state.driving = True
            self.state.operating_mode = AUTOMATIC_OP_MODE

        elif data.status_list[0].status == 3:
            print("Goal has been reached!")
            self.state.driving = False
            self.state.operating_mode = AUTOMATIC_OP_MODE

    def subscribe(self, manufacturer = '', serial_no = ''):
        # subscribers
        rospy.Subscriber('/' + manufacturer + '/' + serial_no + '/move_base/status', GoalStatusArray, callback=self.robot_status, queue_size=None)
        rospy.Subscriber('/' + manufacturer + '/' + serial_no + '/move_base/feedback', MoveBaseActionFeedback, callback=self.robot_feedback, queue_size=None)
        rospy.Subscriber('/' + manufacturer + '/' + serial_no + '/odom', Odometry, callback=self.robot_odom, queue_size=None)

    def execute_order(self):
        pass


    def new_order(self, order):
        pass


    def change_order(self, order):
        
        # update the state header based on the incoming order
        self.state.header_id = self.header_id
        self.state.order_id = order.orderId
        self.state.order_update_id = order.orderUpdateId
        self.state.manufacturer = order.manufacturer
        self.state.serial_number = order.serialNumber
        self.state.timestamp = datetime.datetime.now().isoformat() # This will be updated everytime message is sent

        # populate the actionStates, nodeStates and edgeStates
        action_states_temp = []
        node_states_temp = []
        for node_num, node in zip(range(len(order.nodes)), order.nodes):
            node_dict = {'edgeId': node.nodeId, 'sequenceId': node.sequenceId, 'released': node.released}
            node_states_temp.append(copy.deepcopy(node_dict))
        
            action_state_temp = []
            for action in node.actions:                    
                action_dict = {'actionType': action.actionType, 'actionId': action.actionId, 'actionStatus': self.state.action_states}
                action_state_temp.append(copy.deepcopy(action_dict))
            action_states_temp.append(action_state_temp)

        edge_states_temp = []
        for edge in order.edges:
            edge_dict = {'edgeId': edge.edgeId, 'sequenceId': edge.sequenceId, 'released': edge.released}
            edge_states_temp.append(copy.deepcopy(edge_dict))
        
        self.state.action_states = action_states_temp
        self.state.node_states = node_states_temp
        self.state.edge_states = edge_states_temp

        print(self.state)

        self.execute_order()

        return "The order has been accepted and executed"

    def exec_order(self, order):
        sorted_nodes = sorted(order.nodes, key = lambda i: i['sequenceId']) # sort the nodes based on the sequence id in which they have to be ordered.
        for node, node_num in zip(sorted_nodes, range(len(sorted_nodes))):
            if node.released == False: return # ensuring that the nodes are released before continuing

            print("This is the current node: ", node)

            for action_num in range(len(sorted_nodes[node_num].actions)):
                print("This is the actions for node_num ", node_num, ":", sorted_nodes[node_num].actions[action_num].actionType)

    def order_handle(self, order):

        order_mapped = DotMap(order)

        if order_mapped.orderId != self.state.order_id:
            if self.order_active == True: # Accept order
                if self.state.last_node_id == order_mapped.nodes[0].nodeId: # Check if the start of the new base is the end of the current base
                    status = self.change_order(order_mapped)
                    return status
                else:
                    return "orderUpdateError"

            elif self.order_active == False: # Accept order only if
                # if the robot is within the deviation range of the first node it'll accept the order
                if abs(self.state.agv_position['x'] - order_mapped.nodes[0].nodePosition) < order.nodes[0].nodePosition.allowedDeviationXy and abs(self.state.agv_position['y'] - order_mapped.nodes[0].nodePosition) < order.nodes[0].nodePosition.allowedDeviationXy:
                    status = self.new_order(order_mapped)
                    return status
                else:
                    return "noRouteError"
        else:
            if self.order_update_id > order_mapped.orderUpdateId:
                return "orderUpdateError"
            elif self.order_update_id == order_mapped.orderUpdateId:
                return "Same order. Discarding message!"
            else:
                if self.order_active == True: # Accept order
                    pass
                elif self.order_active == False: # Accept order only if
                    pass 

        # USE list.pop(0) to remove the first element of the list


    def instant_action_handle(self, instant_order):
        pass
    
    def mqtt_cb(self, mqttself, client, message):

        # possibly add decisions to cancel current orders for instant actions.
        if message.topic == ORDER_TOPIC:

            if self.verify_message(json.loads(message.payload.decode('utf8')), ORDER_SCHEMA) == False: return # If the message isn't formatted correctly it will be discarded

            status = self.order_handle(json.loads(message.payload.decode('utf8')))

            print(status)

        elif message.topic == INSTANT_ACTION:

            if self.verify_message(json.loads(message.payload.decode('utf8')), INSTANT_ACTION_SCHEMA) == False: return # If the message isn't formatted correctly it will be discarded

            status = self.instant_action_handle(json.loads(message.payload.decode('utf8')))

            print(status)

    def verify_message(self, msg_list, schema_list, from_file = False):
        """This function verifies the message generated when an order is created.

        Parameters
        ----------
        msg_list : List[dict]
            - This is a list of the json dicts that has to be sent as messages. 
            
        schema_list : List[dict]
            - This is a list of json schemas that correspond to the msg_list. These are used to verify the message is setup correctly.

        Returns
        -------
        is_valid : bool
            - If true, the message is the correct format and the entries are of the right types. If not it will return false.
        """

        if type(msg_list) != list:
            msg_list = [msg_list]
        if type(schema_list) != list:
            schema_list = [schema_list]
    
        if from_file == True:
            for msg, schema in zip(msg_list, schema_list):
                with open('/home/robotlab/turtlebot_ws/src/VDA5050_to_ROS//messages/' + msg, 'r',  encoding = 'utf8') as file:
                    msg_data = json.load(file)
                    file.close()
                with open('/home/robotlab/turtlebot_ws/src/VDA5050_to_ROS/schemas/' + schema, 'r',  encoding = 'utf8') as file:
                    schema = json.load(file)
                    file.close()
                is_valid = jsonschema.Draft3Validator(schema = schema).is_valid(msg_data)
                return is_valid
        else:
            for msg, schema in zip(msg_list, schema_list):
                with open('/home/robotlab/turtlebot_ws/src/VDA5050_to_ROS/schemas/' + schema, 'r',  encoding = 'utf8') as file:
                    schema = json.load(file)
                    file.close()
                is_valid = jsonschema.Draft3Validator(schema = schema).is_valid(msg)
                return is_valid

    def publish_msg(self):
        pass

    def pause(self):
        pass

    def play(self):
        pass

    def cancel(self):
        pass

    def wait_for_trigger(self):
        pass

    def wait_time(self):
        pass

    def charge(self):
        pass

    def getState(self):
        pass

####################################### MAIN LOOP ###################################################

def main():

    print("NOTICE: The class is being run directly from the class file")
    instance_tm = taskManager()
    instance_tm.subscribe('polybot', '22566')
    rospy.spin()
    #instance_tm.mqtt_client.loop_forever()
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
