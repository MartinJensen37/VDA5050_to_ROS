# Standard python libraries
from distutils.log import error
import sys
from dataclasses import dataclass
import datetime
import time
import copy
import uuid
import json
import jsonschema
from typing import List
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
            - nodePosition (omitted)
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

@dataclass
class createOrder():
    
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
        with open('test.json', 'r', encoding='utf8') as json_file:
            
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
                node_dict = {'nodeId': node.node_id, 'sequenceId': node.sequence_id, 'released': node.released}
                node_list_json.append(node_dict)
                
                # Fill out the list of desired actions
                action_list_json = []
                for action, action_num in zip(node.action_list, range(len(node.action_list))):                    
                    action_dict = {'actionType': action.action_type, 'actionId': action.action_id, 'blockingType': action.blocking_type}
                    action_list_json.append(copy.deepcopy(action_dict))
                json_object['nodes'] = node_list_json
                json_object['nodes'][node_num]['actions'] = action_list_json
            return json_object
            

class masterControl:
    """This class is used as a master control for the VDA5050 standard.
    Intialize this class with MQTT broker IP address and port number.
    """
          
    def __init__(self, broker_addr = 'localhost', port = 1883) -> None:
        # Initialize and establish connection to MQTT broker 
        broker_address= broker_addr # 192.168.1.127
        self.mqtt_client = mqtt.Client('Master Control') #create new instance
        self.mqtt_client.on_message = self.mqtt_cb #attach function to callback
        result = self.mqtt_client.connect(broker_address, port) #connect to broker
        self.mqtt_client.subscribe([(STATE_TOPIC, 0), (CONNECTION_TOPIC, 0)]) # Subscribe to topics
        self.mqtt_client.loop_start()

        # Initialize IDs for tracking orders and messages
        self.order_id = '0'
        self.header_id = 0 
        self.update_order_id = 0


    def create_order(self, serial_num = '', manufacturer = '', node_list = None):
        """This function creates an order with the following fields filled out from the schema template order.schema:

        """
        if serial_num == '' or manufacturer == '' or node_list == None:
            print("Please define serial number, manufacturer and a list of nodes.")
            return

        crt_order = createOrder(self.header_id, self.order_id, self.update_order_id, serial_num, manufacturer, node_list)

        return crt_order.return_json()

    def publish_order(self, order_out):

        self.mqtt_client.publish(ORDER_TOPIC, order_out)
        print("Done!")
        self.header_id += 1 # Every time an order is sent increment the header id regardless whether the order is received.
        

    def mqtt_cb(self):
        pass

    def verify_schema(self):
        pass

####################################### MAIN LOOP ###################################################

def main():
    instance_mc = masterControl()


    order_temp = instance_mc.create_order('05', 'polybot', [node("warehouse", 0, True, [actions("1", "start"), actions("0", "stop")]), node("via_point", 1, True, [actions("2", "move")]), node("packing", 2, False, [actions("3", "wait")])])

    out_file = open("order_test.json", "w")
    json.dump(order_temp, out_file, indent=6)

    order_out = json.dumps(order_temp)
    instance_mc.publish_order(order_out)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
