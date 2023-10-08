import cv2
import util
import time
import math
import numpy as np
import threading
from copy import deepcopy
from tobiiglassesctrl import TobiiGlassesController

thread_lock = threading.Lock()  #进程锁
thread_exit = False             #进程判断

#用于保存数据的线程
class tobiiThread(threading.Thread):
    def __init__(self, camera_id):
        super(tobiiThread, self).__init__()
        self.camera_id = camera_id
        self.img_height = 480
        self.img_width = 640
        self.frame = np.zeros((1080, 1920, 3), dtype=np.uint8)

    #获取图像，用于计算击打时的位置
    def get_frame(self):
        return deepcopy(self.frame)

    #获取低像素图像，用于显示
    @staticmethod
    def get_frame_simplifed(self):
        simplifed_frame = cv2.resize(self.frame, (self.img_width, self.img_height))
        return deepcopy(simplifed_frame)

    def run(self):
        global thread_exit
        cap = cv2.VideoCapture(self.camera_id)
        while not thread_exit:
            ret, frame = cap.read()
            if ret:
                thread_lock.acquire()
                self.frame = frame
                thread_lock.release()
            else:
                thread_exit = True
        cap.release()
        print("strming stop")

#用于显示图像的线程
class tobiiShow(threading.Thread):
    def __init__(self, tobiiglasses, thread):
        super(tobiiShow, self).__init__()
        self.tobiiglasses = tobiiglasses
        self.thread = thread
        self.gp = np.array([0.0,0.0])
        self.ts = 0
        self.state = True
        self.frame = np.zeros((1080, 1920, 3), dtype=np.uint8)

    def run(self):#参考：live_scene_and_gaze.py
        global thread_exit
        while not thread_exit:
            thread_lock.acquire()
            self.frame = self.thread.get_frame_simplifed(self.thread)
            thread_lock.release()

            gp = self.tobiiglasses.get_data()['gp'].get('gp')
            if (self.tobiiglasses.get_data()['gp'].get('ts') > 0):
                self.gp[0] = util.data_filter(self.gp[0], gp[0])
                self.gp[1] = util.data_filter(self.gp[1], gp[1])
                cv2.circle(self.frame,(int(self.gp[0]*640),int(self.gp[1]*480)), 30, (0,0,255), 3) #根据上面的简化后的图像大小设置的数据

            if(self.frame.any()==0):
                continue
            else:
                cv2.imshow('Video', self.frame)
            if (cv2.waitKey(1) & 0xFF == ord('q')):
                thread_exit = True
                self.state = False
                cv2.destroyAllWindows()

def tobii_init(address):
    tobiiglasses = TobiiGlassesController(address)#to connect?
    print(tobiiglasses.get_battery_status())
    tobiiglasses.start_streaming()
    time.sleep(0.5)
    return tobiiglasses