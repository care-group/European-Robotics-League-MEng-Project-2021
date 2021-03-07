#! /usr/bin/env python

print("Starting semantic.py")

import rospy
import json
from std_msgs.msg import Float64MultiArray, String

''' TODO
5. make my own maps
6. live updating of semantic map
7. make a service to return array of all entries of name
'''

class SemanticToCoords():
    '''
    This class will start a node that will listen for semantic nav goals on /azm_nav/semantic_goal_listener
    and publish the corresponding coordinate goals to /azm_nav/coord_goal_listener
    '''
    def __init__(self):
        # Base node inits
        rospy.loginfo("Initiating semantic_translator_node")
        rospy.init_node('semantic_translator_node')
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)
        # Semantic goal translating inits
        self.semantic_goal_sub = rospy.Subscriber('/azm_nav/semantic_goal_listener', String, self.sem_goal_cb)
        self.coord_goal_pub = rospy.Publisher('/azm_nav/coord_goal_listener', Float64MultiArray, queue_size=1)
        self.goal = Float64MultiArray()
        # Semantic map inits
        self.semantic_map = []
        self.semantic_map_path = r'/workspace/src/nav/nav_tests/maps/semantic.txt' # TODO change this to be passed in from the launch file
        try:
            with open(self.semantic_map_path, "r") as f:
                self.semantic_map = json.loads(f.read())
        except Exception as e:
            rospy.logerr("An error occured while loading the JSON semantic map")
            rospy.logerr(e)
        # Dynamic semantic map inits
        self.semantic_labels_sub = rospy.Subscriber('/azm_nav/semantic_labels_additions', String, self.add_to_semantic_map)
        # subscriber

    def save_semantic_map():
        # TODO saves the current version of the semantic map to the file
        returb

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        save_semantic_map()
    

    
    def publish_once(self, topic, msg, content="message"):
    """
    Adapted from theconstruct's ros in 5 days course
    will keep retrying to publish the message if the publisher has no connections

    Args:
        topic (rospy publisher object): topic object to publish to
        msg (rospy message object): message object to publish 
        content (String): very short description of message for debug purposes
    """
    attempts = 8
    time_multiplier = 1.5
    sleep = 0.2
    rospy.loginfo("Attempting to publish {} to {}".format(content, topic.name))
    while not self.ctrl_c and attempts:
        connections = topic.get_num_connections()
        if not attempts:
            rospy.logwarn("No listeners on {}, {} wasn't sent.".format(topic, content))
            return
        if connections > 0:
            topic.publish(msg)
            rospy.loginfo("Message published to {}".format(topic.name))
            break
        else:
            rospy.loginfo("No subscribers on {}, sleeping.".format(topic.name))
            rospy.sleep(sleep)
            attempts -= 1
            sleep *= time_multiplier

    def sem_goal_cb(self, msg):
        ''' Listens for semantic goals to move to '''
        for entry in self.semantic_map:
            if entry["name"] == msg.data:
                rospy.loginfo("Semantic goal checks out, translating and sending")
                t = entry["coords"]
                self.goal.data = [t[0], t[1], t[2]]
                self.publish_once(self.coord_goal_pub, self.goal, "coordinate goal")
                return
        rospy.loginfo("The label received does not match any entry in the semantic map, ignoring.")
    
    def add_to_semantic_map(self, msg):
        t = json.loads(msg.data)
        if ("name" not in t or
            "coords" not in t or
            "others" not in t):
            rospy.logwarn("Label received does not feature all required keys (name, coords, others), label received: {}".format(t))
        else:
            self.semantic_map.append(t)

    def remove_from_semantic_map():
        # TODO removes specified object from semantic map
        # how to pick and when is TBD
        return
    
    def update_entry_descriptions():
        # TODO looks up an entry
        pass

    def get_entry_by_name(self, name):
        o = []
        for entry in self.semantic_map:
            if entry["name"] == name:
                o.append(entry)
        return o


    


if __name__ == '__main__':
    print("executing semantic.py as main")
    print("Creating SemanticToCoords obj")
    semantic_translator_obj = SemanticToCoords()
    rospy.loginfo("semantic.py is spinning")
    rospy.spin()
