import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import rosparam
import uuid

def movebase_client(goal_id):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    current_goal_pos = rospy.get_param('/goal/'+ goal_id)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = current_goal_pos[0]
    goal.target_pose.pose.position.y = current_goal_pos[1]
    goal.target_pose.pose.orientation.z = current_goal_pos[2]
    goal.target_pose.pose.orientation.w = current_goal_pos[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        node_list = ['warehouse', 'packing', 'manualHandling', 'charger', 'palletizer'] 
        for node in node_list:
            result = movebase_client(node)

            while not(result):
                pass
            print("The result is in!")
            if result:
                rospy.loginfo("Goal execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")