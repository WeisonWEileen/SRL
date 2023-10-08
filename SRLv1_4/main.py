import cv2
import t265
import D435
import util
import time
import vx300
import tobii
import rospy
import msvcrt
import numpy as np
from threading import Thread
from interbotix_xs_modules.arm import InterbotixManipulatorXS



# 若需要控制两个机械臂，打开终端，输入 'roslaunch interbotix_xsarm_dual xsarm_dual.launch'
# 若只需控制一个机械臂，打开终端，输入'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=vx300'

# 此外，打开一个新终端，输入‘roslaunch realsense2_camera rs_t265.launch’

program_stop = False

#定义机械臂到世界坐标系下的齐次变换矩阵
T1 = np.matrix([[1,0,0,0.23],[0,1,0,0],[0,0,1,0.09],[0,0,0,1]],dtype = object)
T2 = np.matrix([[-1,0,0,0.7],[0,-1,0,0],[0,0,1,0.09],[0,0,0,1]],dtype = object)

def main():
    #定义程序所需参数
    address = "192.168.71.50" #设备的IPV4地址（连接上tobii眼动仪后打开wifi设置查看）
    camera_id = "rtsp://%s:8554/live/scene" % address #设备的摄像机地址
    t265_topic = "/camera/odom/sample" #t265话题名称

    #初始化ros节点
    rospy.init_node("xsarm_dual")

    #初始化tobii眼动仪,用于读取眼动数据
    tobiiglasses = tobii.tobii_init(address)

    #定义化摄像机线程，实时获取摄像机图像，降低延迟，修改读取帧率不会影响延迟
    thread_tobii = tobii.tobiiThread(camera_id)
    thread_tobii.run()#线程开始

    #定义t265消息读取线程，实时获取t265位姿   
    thread_t265 = t265.t265Thread(t265_topic)
    thread_t265.run()

    #定义D435i/D435相机线程，获取相机深度图象，点云图象，RGB图象
    thread_D435 = D435.D435Thread()
    thread_D435.run()

    #需要操作的机械臂数量
    user_input = input("请输需要控制的机械臂数量：")
    SRL_number = int(user_input)

    if(SRL_number == 1):
        thread_vx300_1 = vx300.vx300Thread1()
        thread_vx300_1.run()

    elif(SRL_number == 2):
        thread_vx300_1 = vx300.vx300Thread1()
        thread_vx300_1.run()
        thread_vx300_2 = vx300.vx300Thread2()
        thread_vx300_2.run()

    else:
        print("程序暂时仅支持双机械臂，请输入正确的机械臂数量")

    #显示tobii眼动仪获取到的图像
    thread_show = tobii.tobiiShow(tobiiglasses, thread_tobii)
    thread_show.run()
    
    #开始监听t265状态，等待击打指令
    while True:
        a = thread_t265.get_state1()
        b = thread_t265.get_state2()
        if(a):
            position = util.calposition(thread_tobii, thread_show, thread_D435)
            if(position is not None):
                p = np.matrix([[position.x],[position.y],[position.z],[1.0]],dtype = object)
                p = T1@p
                position.x = p[0]
                position.y = p[1]
                position.z = 0.2
                thread_vx300_1.hit(position)
                thread_t265.set_state1(False)

        if(SRL_number == 2):
            if(b):
                position = util.calposition(thread_tobii, thread_show, thread_D435)
                if(position is not None):
                    p = np.matrix([[position.x],[position.y],[position.z],[1.0]],dtype = object)
                    p = T2@p
                    position.x = p[0]
                    position.y = p[1]
                    position.z = 0.2
                    thread_vx300_1.hit(position)
                    thread_t265.set_state2(False)

        if msvcrt.kbhit():
            if ord(msvcrt.getch()) == 113: # ASCII code for 'q' 按下q退出循环
                break




if __name__=='__main__':
    main()