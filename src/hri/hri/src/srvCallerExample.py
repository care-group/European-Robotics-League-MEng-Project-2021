#!/usr/bin/python

import rospy

from std_msgs.msg import String, Empty
from hri.srv import SpeechToText

rospy.init_node('caller')

rospy.wait_for_service('speechInput')
getSpeechInput =rospy.ServiceProxy('speechInput',SpeechToText)

resp = getSpeechInput()
print("Received input from service:")
print(resp.rawText.data)