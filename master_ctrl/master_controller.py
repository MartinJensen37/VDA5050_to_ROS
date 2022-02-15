# Standard python modules
import sys

# Master controller class module
from master_ctrl_class import masterControl, actions, node

# Other modules
import json
import uuid

def state(client, userdata, message):
    print(message.payload.decode('utf8'))

def main():
    # create an instance of the master controller
    instance_mc = masterControl()

    # define header values
    order_id = str(uuid.uuid1()) # Genereate a unique ID for the current order
    serial_number = '05'
    manufacturer = 'polybot'

    # define order/node/action properties (edges will be added later)
    node_id_list = ['warehouse', 'packing', 'manualHandling', 'charger', 'palletizer']
    sequence_ids = [2, 3, 1, 5, 4] # Order in which the nodes have to be visited.
    released_list = [True, True, True, False, False] # the corresponding release_list
    action_type_list = [['stopPause', 'wait'],['waitForTrigger'],['wait'],['cancelOrder'],['startPause']]
    blocking_type_list = [['HARD', 'HARD'],['HARD'],['HARD'],['SOFT'],['SOFT']]

    node_list = []
    for node_num, node_id, released, sequence_id in zip(range(len(node_id_list)), node_id_list, released_list, sequence_ids):
        action_list = []
        for action_type, blocking_type in zip(action_type_list[node_num], blocking_type_list[node_num]): # Grab the values for 
            action_id = str(uuid.uuid1())
            action_item = actions(action_id, action_type, blocking_type)
            action_list.append(action_item)

        node_item = node(node_id = node_id, sequence_id = sequence_id, released = released, action_list = action_list)
        node_list.append(node_item)
    
    # create an order based upon the given lists (a solution for generating these has to be developed.)
    order_temp = instance_mc.create_order(order_id, serial_number, manufacturer, node_list)

    # verify the validity of the message 
    is_valid = instance_mc.verify_message(order_temp, 'order.schema')

    # if the message is valid send it.
    if is_valid == True:
        order_out = json.dumps(order_temp) # Convert JSON dict to JSON string that can later be loaded as a dict in the task manager
        status = instance_mc.publish_order(order_out) # Publish the order
    
    # create an instance of the subscriber that can listen to the state of the robot
    instance_mc.subcriber()
    instance_mc.mqtt_client.on_message = state # define the callback function where the state is decoded and used
    #instance_mc.mqtt_client.loop_forever() # wait for new state messages until the process is closed


if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)