import rospy
import threading
from copy import deepcopy
from nav_msgs.msg import Odometry

thread_lock = threading.Lock()

#三维坐标
class position(object):
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

#欧拉角
class orientation(object):
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class t265Thread(threading.Thread):
    def __init__(self, topic):
        super(t265Thread, self).__init__()
        self.topic = topic
        self.pos = position(0,0,0)
        self.ori = orientation(0,0,0)
        self.state1 = False
        self.state2 = False

    def get_pos(self):
        return self.pos

    def get_ori(self):
        return self.ori

    def get_state1(self):
        return self.state1

    def set_state1(self, state):
        self.state1 = state

    def get_state2(self):
        return self.state2

    def set_state2(self, state):
        self.state2 = state

    @staticmethod
    def check_state1(self):
        if(self.ori.x >= 0.3):
            self.set_state1(True)
        
    @staticmethod
    def check_state2(self):
        if(self.ori.x <= -0.3):
            self.set_state2 (True)

    @staticmethod
    def callback(data, self):
        thread_lock.acquire()
        self.pos.x = data.pose.pose.position.x
        self.pos.y = data.pose.pose.position.y
        self.pos.z = data.pose.pose.position.z
        self.ori.x = data.pose.pose.orientation.x
        self.ori.y = data.pose.pose.orientation.y
        self.ori.z = data.pose.pose.orientation.z
        bool
        a = self.state1
        b = self.state2
        if(not a):
            self.check_state1(self)
        
        if(not b):
            self.check_state2(self)

        thread_lock.release()

    @staticmethod
    def listener(self, topic="/camera/odom/sample"):
        rospy.Subscriber(name = topic, data_class = Odometry, callback = self.callback, callback_args=self)
        # spin() 是让这个节点退出后python的执行才停止,否则就是持续监听
        rospy.spin()
    def run(self):
        self.listener(self, self.topic)
        
