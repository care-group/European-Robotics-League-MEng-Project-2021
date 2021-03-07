#! /usr/bin/env python

print("Starting semantic.py")

import rospy
import json
from std_msgs.msg import Float64MultiArray, String

''' TODO
4. have the movement be cancellable and the robot stopping where it is
5. make my own maps
6. live updating of semantic map
'''

class SemanticToCoords():
    '''
    This class will start a node that will listen for semantic nav goals on /azm_nav/semantic_goal_listener
    and publish the corresponding coordinate goals to /azm_nav/coord_goal_listener
    '''

    def __init__(self):
        rospy.loginfo("Initiating semantic_translator_node")
        rospy.init_node('semantic_translator_node')

        self.semantic_goal_sub = rospy.Subscriber('/azm_nav/semantic_goal_listener', String, self.sem_goal_cb)
        self.coord_goal_pub = rospy.Publisher('/azm_nav/coord_goal_listener', Float64MultiArray, queue_size=1)
        self.goal = Float64MultiArray()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)

        self.semantic_map = []
        self.semantic_map_path = r'/workspace/src/nav/nav_tests/maps/semantic.txt'

        try:
            with open(self.semantic_map_path, "r") as f:
                self.semantic_map = json.loads(f.read())
        except Exception as e:
            rospy.logerr("An error occured while loading the JSON semantic map")
            rospy.logerr(e)

    def save_semantic_map():
        # TODO saves the current version of the semantic map to the file
        return

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        save_semantic_map()
    
    def publish_once_goal(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        rospy.loginfo("Attempting to publish goal")
        while not self.ctrl_c:
            connections = self.coord_goal_pub.get_num_connections()
            if connections > 0:
                self.coord_goal_pub.publish(self.goal)
                rospy.loginfo("Coord goal published to /azm_nav/coord_goal_listener")
                break
            else:
                self.rate.sleep()

    def sem_goal_cb(self, msg):
        ''' Listens for semantic goals to move to '''
        for entry in self.semantic_map:
            if entry["name"] == msg.data:
                rospy.loginfo("Semantic goal checks out, translating and sending")
                t = entry["coords"]
                self.goal.data = [t[0], t[1], t[2]]
                self.publish_once_goal()
                return
        rospy.loginfo("The label received does not match any entry in the semantic map, ignoring.")
    
    def add_to_semantic_map(self, msg):
        # TODO listens for objects to add to the semantic map
        return

    def remove_from_semantic_map():
        # TODO removes specified object from semantic map
        # how to pick and when is TBD
        return
    


if __name__ == '__main__':
    print("executing semantic.py as main")
    print("Creating SemanticToCoords obj")
    semantic_translator_obj = SemanticToCoords()
    rospy.loginfo("semantic.py is spinning")
    rospy.spin()
