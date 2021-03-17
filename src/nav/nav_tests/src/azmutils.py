from math import sqrt
from math import pi
import tf

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