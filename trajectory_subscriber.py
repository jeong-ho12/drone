import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib.axes import Axes
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg    import PointCloud
from geometry_msgs.msg  import Point32
from nav_msgs.msg       import Odometry


class Subtrajectory:
    def __init__(self):
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber('array',Float32MultiArray, self.pose)

        self.pose_x = []
        self.pose_y = []
        self.pose_z = []


    def pose(self,msg):
        self.pose_x.append(msg.data[0])
        self.pose_y.append(msg.data[1])
        self.pose_z.append(msg.data[2])

if __name__ == "__main__":
    k = Subtrajectory()
    print('llklklklklklkkkllk')
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()
    
    fig = plt.figure(figsize = (14,9), dpi = 100)
    ax = fig.add_subplot(projection='3d')
    
    ax.scatter(k.pose_x, k.pose_y,k.pose_z, marker='s', s=10, c='g')
    plt.show()
