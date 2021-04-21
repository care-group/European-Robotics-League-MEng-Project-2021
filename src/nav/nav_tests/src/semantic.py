#!/usr/bin/python2.7

print("Starting semantic.py")

import rospy
import json
import traceback
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage
from azmutils import dynamic_euclid_dist, str_to_obj, obj_to_str, euler_from_quaternion
import tf
import tf2_ros

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
        # Get pose inits
        #self.pose_sub = rospy.Subscriber('/tf', TFMessage, self.pose_cb)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        #self.tfListener = tf.TransformListener()
        self.robot_pose = [0, 0, 0]
        # CLI emu inits
        self.cli_sub = rospy.Subscriber('/azm_nav/cli_semantic', String, self.cli)


    def load_semantic_map(self):
        try:
            with open(self.semantic_map_path, "r") as f:
                self.semantic_map = json.loads(f.read())
                rospy.loginfo("Map loaded from {}".format(self.semantic_map_path))
        except Exception as e:
            rospy.logwarn("An error occured while loading the JSON semantic map: {}".format(e))

    def save_semantic_map(self):
        try:
            with open(self.semantic_map_path, "w") as f:
                f.write(json.dumps(self.semantic_map, indent=4))
                rospy.loginfo("Map written to {}".format(self.semantic_map_path))
        except Exception as e:
            rospy.logwarn("An error occured while saving the JSON semantic map, hope you made a backup. Error: {}".format(e))

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        if not len(self.semantic_map):
            self.save_semantic_map()
    
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
                if "others" in entry and "seen_from" in entry["others"]:
                    rospy.loginfo("Contains seen_from, using that location isntead")
                    t = entry["others"]["seen_from"]
                    self.goal.data = [t[0], t[1], t[2]]
                    self.publish_once(self.coord_goal_pub, self.goal, "coordinate goal")
                    return
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
                         1 if it gets added

        """ 
        # TODO, shouldnt be a problem but eventually this should handle both full string inputs and unicode
        # if this ever is swapped to python 3 then both are considered to be of type str and this should also be changed
        validation_map = {"name":unicode, "coords":list, "type":unicode, "others":dict}
        if not all(i in input and type(input[i]) == validation_map[i] for i in validation_map):
            rospy.logwarn("Label received does not feature all required keys (name:str, type:str, coords:list, others:dict)," +
                  "label received: {}".format(input))
            print([type(input[i]) for i in input])
            return 0
        
        # check if already there
        # this will fail if the coords are empty FIXME add validation
        _t = (0, 1000)
        for entry in self.semantic_map:
            _dist = dynamic_euclid_dist(entry["coords"], input["coords"])
            if entry["name"] == input["name"] and _dist<distance_threshold:
                if _dist<_t[1]:
                    _t = (entry, _dist)
        # and if so remove it
        if _t[0]:
            l = self.semantic_map.remove(_t[0])
            rospy.loginfo("Removed label from map that was considered to be a duplicate due to being to close to the new addition: {}".format(l))
            

        # append pose
        if "pose_when_seen" not in input["others"]:
            input["others"]["pose_when_seen"] = self.robot_pose
        self.semantic_map.append(input)
        rospy.loginfo("New label ({}) added to semantic map.".format(input["name"]))
        return 1

    def remove_from_semantic_map_by_coords(self, coords, distance_threshold=0.2):
        """
        Deletes the entry whose coords are under the distance threshold and closest to the input

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
        # find closest
        _t = (0, 1000)
        for entry in self.semantic_map:
            _dist = dynamic_euclid_dist(entry["coords"], coords)
            if _dist<distance_threshold and _dist<_t[1]:
                _t = (entry, _dist)
        if _t[0]:
            self.semantic_map.remove(_t[0])
            return _t[0]
        return False

    # TODO
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
            int: 0 if failure
            add_new_to_semantic_map() return for success
        """
        t = msg.data.encode('ascii','replace').replace("\\", "")
        input = str_to_obj(t)
        if not input or "others" not in input:
            rospy.logwarn("Semantic: label received did not have minimum requirements")
            rospy.loginfo(input)
            return 0
        
        _pose = self.get_pose("map", "base_link")
        if _pose:
            input["others"]["seen_from"] = _pose
            rospy.loginfo("Attached current pose {} to the label".format(_pose))
        else:
            rospy.loginfo("Failed to obtain pose when adding label to semantic map, not attaching a 'seen_from' to it")
        return self.add_new_to_semantic_map(input)

    # deprecated and doesnt work, ignore
    def pose_cb(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == "odom" and t.header.frame_id == "map":
                translation = t.transform.translation
                rotation = t.transform.rotation
                euler = euler_from_quaternion(rotation)
                self.robot_pose = [translation.x, translation.y, euler[2]]
                #print(self.robot_pose)
    
    def get_pose(self, one, two):
        try:
            print("attempting to get transform from {} to {}".format(one, two))
            #(trans,rot) = self.tfListener.lookup_transform(one, two, rospy.Time(0), rospy.Duration(4))
            transform = self.tfBuffer.lookup_transform(one, two, rospy.Time(0), rospy.Duration(1))
            print("{}, {}".format(transform.transform.translation.x, transform.transform.translation.y))
            print("rotation:{}".format(euler_from_quaternion(transform.transform.rotation)))
            _pose = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                euler_from_quaternion(transform.transform.rotation)[2]]
            return _pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Error when getting pose from {} to {}: {}".format(one, two, e))
    
    # this is for testing purposes pls no judge
    def cli(self, msg):
        try:
            inputs = msg.data.split(' ', 1)
            o = None
            if inputs[0] == "save": self.save_semantic_map()
            elif inputs[0] == "load": self.load_semantic_map()
            elif inputs[0] == "get": o = self.get_entries_by_name(inputs[1])
            # FIXME
            elif inputs[0] == "remove": o = self.remove_from_semantic_map_by_coords(str_to_obj(inputs[1]))
            elif inputs[0] == "pose":
                i, j = inputs[1].split()
                self.get_pose(i, j)
            if type(o) == str:
                print("CLI output: {}".format(o))
            else:
                print("CLI output:")
                print(obj_to_str(o))
        except Exception as e:
            rospy.logwarn("Bad input on CLI: {}".format(traceback.format_exc()))



if __name__ == '__main__':
    print("executing semantic.py as main")
    print("Creating SemanticToCoords obj")
    semantic_translator_obj = SemanticToCoords(r'/workspace/src/nav/nav_tests/maps/semantic.txt')
    rospy.loginfo("semantic.py is spinning")
    rospy.spin()
