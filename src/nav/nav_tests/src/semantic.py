#! /usr/bin/env python

print("Starting semantic.py")

import rospy
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
        self.rate = rospy.Rate(10) # 10hz
        self.semantic_coords = {
            # coordinates in [x, y, w] 
            # where w is the direction it is facing in degrees
            "door" : [-0.416, 0, 30],
            "drawers" : [0.1, 0, 30],
            "full desk" : [1.5, 0, 30],
            "trash" : [2.7, 0, 30],
            "empty desk" : [0, 1.25, 30],
            "passage" : [2.65, 2.2, 30],
            "dude" : [0.4, 2.8, 30],
            "shelves" : [2.25, 4.4, 30]
        }
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
    
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
        if msg.data not in self.semantic_coords:
            rospy.loginfo("Semantic goal received which doesn't not have associated coordinates, ignoring.")
        else:
            rospy.loginfo("Semantic goal checks out, translating and sending")
            t = self.semantic_coords[msg.data]
            self.goal.data = [t[0], t[1], t[2]]
            self.publish_once_goal()


if __name__ == '__main__':
    print("executing semantic.py as main")
    print("Creating SemanticToCoords obj")
    semantic_translator_obj = SemanticToCoords()
    rospy.loginfo("semantic.py is spinning")
    rospy.spin()
