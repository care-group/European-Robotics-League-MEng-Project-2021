#! /usr/bin/env python

print("Starting semantic_copy.py")

import json
from math import sqrt
#from std_msgs.msg import Float64MultiArray, String

''' TODO
5. make my own maps
6. live updating of semantic map
7. make a service to return array of all entries of name
'''

def dynamic_euclid_dist(a, b):
    o = 0
    for i in range(len(a)):
        o += (a[i]-b[i])**2
    return sqrt(o)

class SemanticToCoords():
    '''
    This class will start a node that will listen for semantic nav goals on /azm_nav/semantic_goal_listener
    and publish the corresponding coordinate goals to /azm_nav/coord_goal_listener
    '''
    def __init__(self, map_path=None):
        # Base node inits
        #rospy.loginfo("Initiating semantic_node")
        #rospy.init_node('semantic_node')
        self.ctrl_c = False
        #self.rate = rospy.Rate(10)
        #rospy.on_shutdown(self.shutdownhook)
        # Semantic goal translating inits
        #self.semantic_goal_sub = rospy.Subscriber('/azm_nav/semantic_goal_listener', String, self.sem_goal_cb)
        #self.coord_goal_pub = rospy.Publisher('/azm_nav/coord_goal_listener', Float64MultiArray, queue_size=1)
        #self.goal = Float64MultiArray()
        # Semantic map inits
        self.semantic_map = []
        self.semantic_map_path = map_path # TODO change this to be passed in from the launch file
        if (map_path is not None):
            self.load_semantic_map()
        # Dynamic semantic map inits
        #self.semantic_labels_sub = rospy.Subscriber('/azm_nav/semantic_label_additions', String, self.add_to_semantic_map)
        # subscriber

    def load_semantic_map(self):
        try:
            with open(self.semantic_map_path, "r") as f:
                self.semantic_map = json.loads(f.read())
        except Exception as e:
            # TODO make rospy.log
            print("An error occured while loading the JSON semantic map")
            print(e)

    def save_semantic_map(self):
        try:
            with open(self.semantic_map_path, "w") as f:
                f.write(json.dumps(self.semantic_map, indent=4))
        except Exception as e:
            # TODO make rospy.log
            print("An error occured while saving the JSON semantic map, hope you made a backup")
            print(e)

    def string_to_obj(self, string):
        return json.loads(string)
    
    def obj_to_string(self, obj):
        return json.dumps(obj)

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        print("Semantic shutting down, saving")
        self.save_semantic_map()
    
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
        validation_map = {"name":str, "coords":list, "type":str, "others":dict}
        if not all(i in input and type(input[i]) == validation_map[i] for i in validation_map):
            # TODO make rospy.log
            print("Label received does not feature all required keys (name:str, type:str, coords:list, others:dict)," +
                  "label received: {}".format(input))
            return 0
        for entry in self.semantic_map:
            if entry["name"] == input["name"] and dynamic_euclid_dist(entry["coords"], input["coords"])<distance_threshold:
                # Assume it's already here
                return 2
        self.semantic_map.append(input)
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
            # TODO make rospy.log
            print("Bad input when trying to delete from semantic map")
            return False
        for entry in self.semantic_map:
            if dynamic_euclid_dist(entry["coords"], coords)<distance_threshold:
                self.semantic_map.remove(entry)
                return entry
        return False

    # TODO looks up an entry
    def update_entry_descriptions(self):
        pass

    # TODO Needs docstring
    def get_entries_by_name(self, name):
        o = []
        for entry in self.semantic_map:
            if entry["name"] == name:
                o.append(entry)
        return o


    
if __name__ == '__main__':
    print("executing semantic.py as main")
    print("Creating SemanticToCoords obj")
    sem = SemanticToCoords(r'C:\Users\AZM\Documents\ERL\european_robotic_league\src\nav\nav_tests\maps\semantic.txt')
    
    # rospy.loginfo("semantic.py is spinning")
    # rospy.spin()
