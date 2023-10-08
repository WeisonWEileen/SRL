# 此代码是第一版的机械臂打地鼠代码
#
# 首先，打开终端，并输入'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=vx300'
# 然后打开次文件对应的文件夹，在终端输入 'python graduate_design.py'
#此段代码用于实现整个操作流程
import cv2
import json
import time
import math
import rospy
import pyzbar
import numpy as np
import threading
from copy import deepcopy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tobiiglassesctrl import TobiiGlassesController
from interbotix_xs_modules.arm import InterbotixManipulatorXS

thread_lock = threading.Lock()
thread_exit = False
gp_x = 0.0
gp_y = 0.0


class myThread(threading.Thread):
    def __init__(self, camera_id):
        super(myThread, self).__init__()
        self.camera_id = camera_id
        self.img_height = 480
        self.img_width = 640
        self.frame = np.zeros((1080, 1920, 3), dtype=np.uint8)

    def get_frame(self):
        return deepcopy(self.frame)

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

class position(object):
    def __init__(self, rot=0, x=0, y=0, z=0):
        self.rot = rot
        self.x = x
        self.y = y
        self.z = z

def robot_1_init(robot_1):
    # robot_1 = InterbotixManipulatorXS(robot_model="vx300", robot_name="arm_1", moving_time=1.0, gripper_pressure=1.0, init_node=False)
    robot_1.arm.go_to_sleep_pose(moving_time=2) #Initiallize the robot postition
    robot_1.arm.go_to_home_pose(moving_time=2)  #set the arm to the home pose
    robot_1.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=2) #go to the position where the robot is convinent to hit
    return robot_1

def robot_2_init(robot_2):
    
    robot_2.arm.go_to_sleep_pose(moving_time=2) #Initiallize the robot postition
    robot_2.arm.go_to_home_pose(moving_time=2)  #set the arm to the home pose
    robot_2.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=2) #go to the position where the robot is convinent to hit
    return robot_2

def callback(data):
    print("b")
    print(data)

def listener():
    # 在ros中，nodes 是唯一命名的。如果被运行的两个节点有相同的名字，则之前运行的将会被踢掉
    # anonymous=True 标志意味着对于我们的“listener” node ， rospy 将会选择一个唯一的名字
    # 以便多个 listeners 可以同时运行
    print("a")
    rospy.Subscriber("/camera/odom/sample",Odometry, callback)

     # spin() 是让这个节点退出后python的执行才停止,否则就是持续监听
    rospy.spin()


#获取目标击打位置的方法
def get_target_position(gp_data, img):
    
    #对获取到的图像进行处理
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    size = gray_img.shape[::-1]
    gpx = 0
    gpy = 0
    i = 1
    # print(size[0])
    # print(gp_data[-1][0]*size[0])
    # gaze_pixel = np.array([np.mean(gp_data[-10:][0])*size[0]],[np.mean(gp_data[-10:][1])*size[1]],[1])
    while(np.abs(gpx-gp_data[-i][0])>=0.05 or np.abs(gpy-gp_data[-i][1])>=0.05 or (gp_data[-i][0] <= 0.05 and gp_data[-i][1] <= 0.05)):
        gpx = gp_data[-i][0]
        gpy = gp_data[-i][1]
        i = i+1

    gaze_pixel = np.array([[gpx*size[0]], [gpy*size[1]], [1]])
    qrCodeDetector = cv2.QRCodeDetector()
    retval, points = qrCodeDetector.detect(gray_img)
    
    if(not retval):
        img1 = cv2.GaussianBlur(gray_img,(1,1),2,2,cv2.BORDER_DEFAULT)
        retval, points = qrCodeDetector.detect(img1)
        if(not retval):
            retval,img1 = cv2.threshold(img1,0,255,cv2.THRESH_OTSU)
            retval, points = qrCodeDetector.detect(img1)
            if(not retval):
                element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
                img1 = cv2.morphologyEx(img1,cv2.MORPH_OPEN,element)
                img1 = cv2.morphologyEx(img1,cv2.MORPH_CLOSE,element)
                retval, points = qrCodeDetector.detect(img1)
                if(not retval):
                    print("can not find QR code")
    print(points)
    # print(points[0])
    # print(points[0][0])
    # print(points[0][0][0])
    # print([points[0][1][0],points[0][1][1]])

    if(points is not None):
        imagePoints = np.array([[points[0][0][0],points[0][0][1]],[points[1][0][0],points[1][0][1]],[points[2][0][0],points[2][0][1]],[points[3][0][0],points[3][0][1]]]).astype(float) #像素坐标,单位:像素
        objectPoints = np.array([[0,0,0],[0,49,0],[49,49,0],[49,0,0]]).astype(float) #空间坐标,单位:毫米mm
        focal_length = 1.123161286201691e+03 #相机焦距
        cameraMatrix = np.array([[focal_length, 0, 9.470499505963581e+02],[0, focal_length, 5.492222351061334e+02],[0, 0, 1]]).astype(float) #相机内参矩阵
        distCoeffs = np.array([-0.002877288092662,0.032719266686277,0,0]).astype(float)

        #计算相机空间位置，retval:结果，true/false rvec:旋转向量，方向为旋转轴，模值为旋转角，tvec：平移向量 flags:求解方法，详见官方文档，SOLVEPNP_ITERATIVE此方法需要至少4对点
        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_ITERATIVE) 
        R = cv2.Rodrigues(rvec)[0] #利用罗德里格斯公式求旋转矩阵,输出为R，jacobian
        
        #计算注视点
        point = trans_2D_to_3D(R, tvec, cameraMatrix, gaze_pixel)
        print("world point is:\n",point.reshape(1,3))

        target_position = position()
        target_position.rot = math.atan2(point[1],point[0])
        # target_position.x = math.sqrt(point[0]**2+point[1]**2)/1000
        target_position.x = point[0]/1000+0.25
        target_position.y = point[1]/1000+0.05
        target_position.z = 0.1
        return target_position
    else:
        print("falile to detect the QR code")
        return None

#获取脚部状态的方法
def get_heel_status(gp_data):
    # count = 0
    # for i in range(1,70):
    #     if(gp_data[-1][0]<= 0.05 and gp_data[][1] <= 0.05):
    #         count = count + 1
    if(len(gp_data)%1000 == 0):
        return True
    
    else:
        return False

# #判断是否结束打地鼠程序
# def get_program_status():
#     if(True):
#         return True
    
#     else:
#         return False

# #二维码识别
# def decode_display(image):
#     barCodes = pyzbar.decode(image)

#     for barCode in barCodes:
#         (x,y,w,h) = barCode.rect()

#已知像素点坐标和空间z平面以及相机内参矩阵，平移旋转向量求三维空间坐标
#R:旋转矩阵np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])，t平移向量np.array([[tx],[ty],[tz]])， 
# cameraMatrix:相机内参矩阵np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]), imagePoints:像素坐标np.array([[ux],[vy],[1]])，
# z:世界坐标系z值，默认为0
def trans_2D_to_3D(R, t, cameraMatrix, imagePoints, z=0):
    s = (z + (np.linalg.inv(R)@t)[2])/((np.linalg.inv(R)@np.linalg.inv(cameraMatrix)@imagePoints)[2]) #计算深度
    position = np.linalg.inv(R)@(s*np.linalg.inv(cameraMatrix)@imagePoints - t)#计算像素点对应的世界坐标系位置
    position[2] = z
    return position
    


def main():

    bot = InterbotixManipulatorXS(robot_model="vx300", group_name="arm", gripper_name="gripper", robot_name="vx300", moving_time=0.5, accel_time=0.2, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=True)
    bot.arm.go_to_sleep_pose(moving_time=2) #Initiallize the robot postition
    bot.arm.go_to_home_pose(moving_time=2)  #set the arm to the home pose
    bot.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=2) #go to the position where the robot is convinent to hit

    #获取眼动仪数据
    address = "192.168.71.50"
    tobiiglasses = TobiiGlassesController(address)
    print(tobiiglasses.get_battery_status())
    tobiiglasses.start_streaming()
    time.sleep(0.5)

    global thread_exit
    camera_id = "rtsp://%s:8554/live/scene" % address
    thread = myThread(camera_id)
    thread.start()
    threading.Thread(target=listener).start()
    cap = cv2.VideoCapture(camera_id)
    if (cap.isOpened()== False):
        print("Error opening video stream or file")

    bool_raise_heel = False #判断一轮中是否抬起脚跟表示进行击打
    #开始一轮的操作流程，首先，机械臂归位，眼动仪开始获取用户注释信息，当用户抬脚后，机械臂开始运动，击打目标，结束一轮
    gp_data = []
    while(cap.isOpened()):
        
        while not thread_exit:
            thread_lock.acquire()
            frame = thread.get_frame_simplifed()
            thread_lock.release()


            height, width = frame.shape[:2]
            gp = tobiiglasses.get_data()['gp'].get('gp')
            global gp_x,gp_y
            if (tobiiglasses.get_data()['gp'].get('ts') > 0):
                gp_x = (1-np.exp(-1.0/3.0))*gp[0] + np.exp(-1.0/3.0)*gp_x
                gp_y = (1-np.exp(-1.0/3.0))*gp[1] + np.exp(-1.0/3.0)*gp_y #滤波，加权滤波，权重指数衰减
                cv2.circle(frame,(int(gp_x*width),int(gp_y*height)), 30, (0,0,255), 3)
                
                if(not bool_raise_heel): #判断用户是否抬起脚跟执行击打操作
                    if(not gp == None):  #判断眼动仪是否成功获取注视点位置，用于之后的多目标击打判断，暂时的实现方式可以是利用抬脚的一瞬间获取眼睛的注视位置
                        gp_data.append(np.array([gp_x,gp_y]))  #数据类型处理为np.array()与之后相统一
                    bool_raise_heel = get_heel_status(gp_data)
                    # time.sleep(0.1)#以10Hz采样，暂定这个数，眼动仪采样频率好像是50Hz，不知道归不会有影响
                        
                else:
                    frame = thread.get_frame()
                    target_position = get_target_position(gp_data, frame)#通过上述的数据计算击打位置
                    if(target_position is not None):
                        bot.arm.set_ee_pose_components(x=target_position.x, y=target_position.y, z=target_position.z, moving_time=1) #hit the target position
                        time.sleep(3)
                        bot.arm.set_joint_positions([0.0, -0.4, 1.0, -0.6, 0.0], moving_time=1)  #go to the position where the robot is convinent to hit
                    bool_raise_heel = False
                    gp_data.clear()

            if(frame.any()==0):
                continue
            else:
                cv2.imshow('Video', frame)
            if (cv2.waitKey(1) & 0xFF == ord('q')):
                thread_exit = True

        else:
            break


    cap.release()
    cv2.destroyAllWindows()
    tobiiglasses.stop_streaming()
    tobiiglasses.close()
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    thread.join()
    print("program finish")


if __name__=='__main__':
    main()