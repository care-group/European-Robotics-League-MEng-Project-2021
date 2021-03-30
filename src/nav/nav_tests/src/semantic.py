#!/usr/bin/python2.7

print("Starting semantic.py")

import rospy
import json
from std_msgs.msg import Float64MultiArray, String
from azmutils import dynamic_euclid_dist, str_to_obj, obj_to_str

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
    def __init__(self, map_path=None):
        # Base node inits
        rospy.loginfo("Initiating semantic_node")
        rospy.init_node('semantic_node')
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)
        # Semantic goal translating inits
        self.semantic_goal_sub = rospy.Subscriber('/azm_nav/semantic_goal_listener', String, self.sem_goal_cb)
        self.coord_goal_pub = rospy.Publisher('/azm_nav/coord_goal_listener', Float64MultiArray, queue_size=1)
        self.goal = Float64MultiArray()
        # Semantic map inits
        self.semantic_map = []
        self.semantic_map_path = map_path # TODO change this to be passed in from the launch file
        if (self.semantic_map_path is not None):
            self.load_semantic_map()
        # Dynamic semantic map inits
        self.semantic_labels_sub = rospy.Subscriber('/azm_nav/semantic_label_additions', String, self.add_to_semantic_map_callback)
        # Get pose inits TODO
        # self.pose_sub = rospy.Subscriber('', Pose, self.pose_cb)
        self.robot_pose = []


    def load_semantic_map(self):
        try:
            with open(self.semantic_map_path, "r") as f:
                self.semantic_map = json.loads(f.read())
        except Exception as e:
            rospy.logwarn("An error occured while loading the JSON semantic map: {}".format(e))

    def save_semantic_map(self):
        try:
            with open(self.semantic_map_path, "w") as f:
                f.write(json.dumps(self.semantic_map, indent=4))
        except Exception as e:
            rospy.logwarn("An error occured while saving the JSON semantic map, hope you made a backup. Error: {}".format(e))

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        if not len(self.semantic_map):
            self.save_semantic_map()
            rospy.loginfo("Saved map before shutting down")
    
    def publish_once(self, topic, msg, content="message"):
        '''
        Adapted from theconstruct's ros in 5 days course
        will keep retrying to publish the message if the publisher has no connections

        Args:
            topic (rospy publisher object): topic object to publish to
            msg (rospy message object): message object to publish 
            content (String): very short description of message for debug purposes
        '''
        attempts = 8
        time_multiplier = 1.3
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

    def add_new_to_semantic_map(self, input, distance_threshold=0.1):
        """
        Adds a new label object to the semantic map,
        input dict must contain minimum {"name":str, "coords":list, "type":str, "others":dict}
        but only the coords need to be populated
        Args:
            input (dict): a dictionary representing a label object in the semantic map

        Returns:
            int: returns 0 if the input did not pass validation checks, 
                         1 if it gets added,
                         2 if it didn't get added because an item under the threshold already exists

        """ 
        # TODO, shouldnt be a problem but eventually this should handle both full string inputs and unicode
        # if this ever is swapped to python 3 then both are considered to be of type str and this should also be changed
        validation_map = {"name":unicode, "coords":list, "type":unicode, "others":dict}
        if not all(i in input and type(input[i]) == validation_map[i] for i in validation_map):
            rospy.logwarn("Label received does not feature all required keys (name:str, type:str, coords:list, others:dict)," +
                  "label received: {}".format(input))
            print([type(input[i]) for i in input])
            return 0
        for entry in self.semantic_map:
            if entry["name"] == input["name"] and dynamic_euclid_dist(entry["coords"], input["coords"])<distance_threshold:
                # Assume it's already here
                return 2
        self.semantic_map.append(input)
        rospy.loginfo("New label ({}) added to semantic map.".format(input["name"]))
        return 1

    # TODO make it iterate thru the entire input and delete the one whose coords are the closest it
    # TODO refactor to use integer retursns like in the add method
    def delete_from_semantic_map_by_coords(self, coords, distance_threshold=0.2):
        """
        Deletes the first entry whose coords are under the distance threshold

        Args:
            coords (list, tuple): list or tuple with 3 ints/floats describing the position of the label to be deleted
            distance_threshold (float, optional): the threshold with which to accept a coord as matching that of the label. Defaults to 0.2.

        Returns:
            False, dict: Returns the removed dict if it succeeds, returns False otherwise
        """
        if not (isinstance(coords, (list, tuple)) and
           all(isinstance(i, (int, float)) for i in coords) and
           len(coords) == 3):
            rospy.logwarn("Bad input when trying to delete from semantic map")
            return False
        for entry in self.semantic_map:
            if dynamic_euclid_dist(entry["coords"], coords)<distance_threshold:
                self.semantic_map.remove(entry)
                return entry
        return False

    def update_entry_descriptions(self):
        pass

    # TODO Needs docstring
    def get_entries_by_name(self, name):
        o = []
        for entry in self.semantic_map:
            if entry["name"] == name:
                o.append(entry)
        return o

    def add_to_semantic_map_callback(self, msg):
        """
        Callback function for receiving labels
        attaches the robot's current pose to the input

        Returns:
            int: 0 for failure
                 1 for success
        """
        t = msg.data.encode('ascii','replace').replace("\\", "")
        input = str_to_obj(t)
        if not input or "others" not in input:
            rospy.logwarn("Semantic: label received did not have minimum requirements")
            rospy.loginfo(input)
            return 0
        input["others"]["seen_from"] = self.robot_pose
        self.add_new_to_semantic_map(input)
        return 1

    # TODO
    def pose_cb(self, msg):
        pass
    


if __name__ == '__main__':
    print("executing semantic.py as main")
    print("Creating SemanticToCoords obj")
    semantic_translator_obj = SemanticToCoords(r'/workspace/src/nav/nav_tests/maps/semantic.txt')
    rospy.loginfo("semantic.py is spinning")
    rospy.spin()
