# Standard python libraries
from distutils.log import error
import sys
from dataclasses import dataclass, field
import datetime
import time
import copy
import uuid
import json
import jsonschema
from typing import List, Dict
from mysqlx import DataError
# MQTT paho specific modules
import paho.mqtt.client as mqtt

########################## GLOBAL VARIABLES AND CONSTANTS ###########################################

# These will be updated at major/minor changes e.g. x.x.0 when the x's are changing
ORDER_TOPIC = 'IS/v0.1.0/polybot/05/order'
STATE_TOPIC = 'IS/v0.1.0/polybot/05/state'
CONNECTION_TOPIC = 'IS/v0.1.0/polybot/05/connection'

HEADER_LIST = ['headerId', 'orderId', 'orderUpdateId', 'manufacturer']

########################## CLASSES AND FUNCTION DEFINITIONS #########################################
@dataclass
class actions():
    """This class constructs the action list element in the json message 
        - action:
            - actionType
            - actionId
            - blockingType (only HARD is included in the types of actions in the current demo)
            - actionParameters (currently omitted for simplicity)
    """

    action_id: str = None
    action_type: str = None
    blocking_type: str = 'HARD'

@dataclass
class node():
    """This class constructs the node list element in the json message
        - node:
            - nodePosition (omitted but set to zero)
            - action:
                - actionType
                - actionId
                - blockingType (only HARD is included in the types of actions in the current demo)
                - actionParameters (currently omitted for simplicity)
            - edge (omitted since we only have 1 node)
    """

    node_id: str = None
    sequence_id: int = None
    released: bool = None
    action_list: List[actions] = None
    node_pos: Dict[float, str] = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'allowed_div_xy': 0.0, 'allowed_div_theta': 0.0, 'map_id': '', 'map_description': ''})
    
@dataclass
class order():
    """This class constructs an order with the following fields filled out from the schema template order.schema:
            - header:
                - headerId
                - orderId
                - orderUpdateId
                - nodes (1 node is sent in array format)
                - edges = 0 (currently)
            - node:
                - nodePosition (omitted)
                - action:
                    - actionType
                    - actionId
                    - blockingType (only HARD is included in the types of actions in the current demo)
                    - actionParameters (currently omitted for simplicity)
                - edge (omitted since we only have 1 node)
    """

    header_id: int = None
    order_id: str = None
    order_update_id: int = None
    serial_number:str = None
    manufacturer: str = None
    nodes: List[node] = None

    def return_json(self):
        with open('../messages/order_msg.json', 'r', encoding='utf8') as json_file:
            
            # Open json schema as a template for creating the order.
            json_object = json.load(json_file)
            json_file.close()

            # Fill out the header for the order using the following parameters
            json_object['headerId'] = self.header_id
            json_object['orderId'] = self.order_id
            json_object['orderUpdateId'] = self.order_update_id
            json_object['manufacturer'] = self.manufacturer
            json_object['serialNumber'] = self.serial_number
            json_object['timestamp'] = datetime.datetime.now().isoformat()
            
            # Fill out the list of nodes the robot has to move through
            node_list_json = []
            for node, node_num in zip(self.nodes, range(0, len(self.nodes))):
                node_dict = {'nodeId': node.node_id, 'sequenceId': node.sequence_id, 'released': node.released, 'nodePosition':node.node_pos}
                node_list_json.append(node_dict)
                
                # Fill out the list of desired actions
                action_list_json = []
                for action in node.action_list:                    
                    action_dict = {'actionType': action.action_type, 'actionId': action.action_id, 'blockingType': action.blocking_type}
                    action_list_json.append(copy.deepcopy(action_dict))
                json_object['nodes'] = node_list_json
                json_object['nodes'][node_num]['actions'] = action_list_json
            return json_object
            

class masterControl:
    """This class is used as a master control for the VDA5050 standard.
    Intialize this class with MQTT broker IP address and port number.

        - Inputs:
            - broker_addr : str, IP address of the MQTT broker
            - port : int, port number of the MQTT broker
    """
          
    def __init__(self, broker_addr = 'localhost', port = 1883) -> None:
        # initialize and establish connection to MQTT broker 
        self.mqtt_client = mqtt.Client('Master Control') #create new instance of the mqtt node
        self.mqtt_client.on_message = self.mqtt_cb # define default callback function
        result = self.mqtt_client.connect(broker_addr, port) #connect to broker

        # initialize IDs for tracking orders and messages
        self.order_id = ''
        self.order_id_backlog = []
        self.header_id = 0 
        self.update_order_id = 0

    def create_order(self, order_id = '', serial_num = '', manufacturer = '', node_list = None):
        """This function creates an order with the following fields filled out from the schema template order.schema:

        """

        if serial_num == '' or manufacturer == '' or node_list == None:
            print("Please define serial number, manufacturer and a list of nodes.")
            return
        
        self.order_id = order_id # create an order with a unique order id

        crt_order = order(self.header_id, self.order_id, self.update_order_id, serial_num, manufacturer, node_list)

        return crt_order.return_json()

    def publish_order(self, order_out, order_topic=ORDER_TOPIC): 
        """This method publishes orders to a given MQTT broker defined when an instance of the master control class is initialized.
        Parameters
        ----------
        order_out : json dict
            - the message is sent as a json dict
        order_topic : str
            - the topic is defined as a string. The format is 'interfaceName/majorVersion/manufacturer/serialNumber/topic'.
            In this case the topic should always be 'order'

        Returns
        -------
        integer regarding the status of the transmission:
            MQTT_ERR_AGAIN = -1 \n
            MQTT_ERR_SUCCESS = 0\n
            MQTT_ERR_NOMEM = 1\n
            MQTT_ERR_PROTOCOL = 2\n
            MQTT_ERR_INVAL = 3\n
            MQTT_ERR_NO_CONN = 4\n
            MQTT_ERR_CONN_REFUSED = 5\n
            MQTT_ERR_NOT_FOUND = 6
            MQTT_ERR_CONN_LOST = 7\n
            MQTT_ERR_TLS = 8\n
            MQTT_ERR_PAYLOAD_SIZE = 9\n
            MQTT_ERR_NOT_SUPPORTED = 10\n
            MQTT_ERR_AUTH = 11\n
            MQTT_ERR_ACL_DENIED = 12\n
            MQTT_ERR_UNKNOWN = 13\n
            MQTT_ERR_ERRNO = 14\n
            MQTT_ERR_QUEUE_SIZE = 15\n
            MQTT_ERR_KEEPALIVE = 16\n
        """

        status = self.mqtt_client.publish(ORDER_TOPIC, order_out)
        self.header_id += 1 # Every time an order is sent increment the header id regardless whether the order is received.

        return status.rc
    
    
    def subcriber(self):
        """this function subscribes to the 'connection' and 'state' topics with a qos of 0 as defined in VDA5050
        """

        self.mqtt_client.subscribe([(STATE_TOPIC, 0), (CONNECTION_TOPIC, 0)]) # Subscribe to topics

    
    def mqtt_cb(self, mqttself, client, message):
        """this function is only used for debug purposes

        Parameters
        ----------
        message : any
            - returns whatever message is incoming. This is often times return as a bytearray that has to be decoded.
        Returns
        -------
        message : any
            - this function simply return the message when received
        """

        print(message)

        return message

    # this function verifies the message generated
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
                with open('../messages/'+msg, 'r',  encoding='utf8') as file:
                    msg_data = json.load(file)
                    file.close()
                with open('../schemas/'+schema, 'r',  encoding='utf8') as file:
                    schema = json.load(file)
                    file.close()
                is_valid = jsonschema.Draft3Validator(schema=schema).is_valid(msg_data)
                return is_valid
        else:
            for msg, schema in zip(msg_list, schema_list):
                with open('../schemas/'+schema, 'r',  encoding='utf8') as file:
                    schema = json.load(file)
                    file.close()
                is_valid = jsonschema.Draft3Validator(schema=schema).is_valid(msg)
                return is_valid

    def save_message_json(self, msg_data = None, msg_name="default.json"):
        """This function saves a generated msg as a JSON file

        Parameters
        ----------
        msg_data : json dict
            - json message that has to be saved.

        msg_name : str
            - define a filename for the message, by default "default.json"
            - include ".json" in the filename
        """

        if msg_data == None:
            print("Please specify a messages to save")
            return

        out_file = open(msg_name, "w")
        print(type(msg_data))
        json.dump(msg_data, out_file, indent=6)

        return "success"

####################################### MAIN LOOP ###################################################

def main():

    print("NOTICE: The class is being run directly from the class file")
    instance_mc = masterControl()

    order_temp = instance_mc.create_order('test', '05', 'polybot', [node("warehouse", 0, True, [actions("1", "start"), actions("0", "stop")]), node("via_point", 1, True, [actions("2", "move")]), node("packing", 2, False, [actions("3", "wait")])])

    order_out = json.dumps(order_temp) # Convert JSON dict to JSON string that can later be loaded as a dict in the task manager
    instance_mc.save_message_json(order_temp, '../messages/checkJson.json')
    check_message = instance_mc.verify_message('checkJson.json', 'order.schema')
    status =instance_mc.publish_order(order_out) # Publish the order
    print(check_message) # print the status regarding the message.


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
