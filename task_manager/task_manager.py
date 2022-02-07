########################### IMPORTED MODULES AND MESSAGES ###########################################
# Standard python modules
import sys

# JSON specific modules
import json
import jsonschema

# MQTT paho specific modules
import paho.mqtt.client as mqtt

# ROS specific modules
import rospy

# ROS specific messages
from geometry_msgs import Twist, Pose


########################## GLOBAL VARIABLES AND CONSTANTS ###########################################

# These will be updated at major/minor changes e.g. x.x.0 when the x's are changing
ORDER_TOPIC = 'IS/v0.1.0/polybot/05/order'
INSTANT_ACTION = 'IS/v0.1.0/polybot/05/instantAction'
STATE_TOPIC = 'IS/v0.1.0/polybot/05/state'
CONNECTION_TOPIC = 'IS/v0.1.0/polybot/05/connection'

########################## CLASSES AND FUNCTION DEFINITIONS #########################################

class taskManager():

    def __init__(self, broker_addr = '192.168.1.127', port = 1883) -> None:

        # initialize rospy subscribers and publishers
        rospy.init_node("service_control_node")
        rospy.Subscriber('/', Twist, callback=self.robot_cb, queue_size=None)

        # initialize and establish connection to MQTT broker 
        self.mqtt_client = mqtt.Client('Task Manager') #create new instance of the mqtt node
        self.mqtt_client.on_message = self.mqtt_cb # define default callback function
        result = self.mqtt_client.connect(broker_addr, port) #connect to broker

    def robot_cb(self):
        pass
    
    def mqtt_cb(self):
        pass

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


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)