from math import sqrt
from math import pi
import json
import tf
from geometry_msgs.msg import Quaternion

def dynamic_euclid_dist(a, b):
    o = 0
    for i in range(len(a)):
        o += (a[i]-b[i])**2
    return sqrt(o)

def quaternion_from_euler(roll, pitch, yaw):
    '''
    From HSR's utils.py
    '''
    q = tf.transformations.quaternion_from_euler(roll / 180.0 * pi,
                                                 pitch / 180.0 * pi,
                                                 yaw / 180.0 * pi, 'rxyz')
    
    return Quaternion(q[0], q[1], q[2], q[3])

def str_to_obj(string):
    """
    Converts JSON string to data structure

    Args:
        string (str): valid JSON string

    Raises:
        ValueError: if input isnt a valid JSON string

    Returns:
        Data structure: [description]
    """ 
    try:
        return json.loads(string)
    except ValueError as e:
        raise ValueError("ValueError occured when loading JSON string: {}, the input was: {}".format(e, string))
    
def obj_to_str(obj):
    return json.dumps(obj)