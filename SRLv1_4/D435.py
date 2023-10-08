import threading
import math
import cv2
import numpy as np
import pyrealsense2 as rs

class D435Thread(threading.Thread):
    #初始化D435的线程
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True
        #fzz加的
        self.verts = None
        self.out = None
        self.depth_image = None
        self.color_image = None

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))#'_'表示不需要第二个变量就占个位置(wei)
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)

#fzz加的获取函数
    def get_color_frame(self):
        return self.color_image
    
    def get_depth_frame(self):
        return self.depth_image


    def run(self):
        pipeline = rs.pipeline()
        config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(pipeline)#创建realsense管道对象
        pipeline_profile = config.resolve(pipeline_wrapper)#解析配置并创建管道配置文件
        device = pipeline_profile.get_device()#获取与管道相关联的设备对象
        
        #检测是否有RGB摄像头  （fzz)
        #怎么不加上是否有关深度的识别？（wei）
        #我手动加上了(9/22/2023),我也不懂为什么
        found_rgb = False

        #相机的RGB和Depth功能的检测,对于D435i，会自动先识别Stereo Module
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'Stereo Module':
                print('depth_connected_successfully')
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                print('RGB_connected_successfully')
                found_rgb = True
                break
        if not found_rgb:#若没有同时识别两种Module，报错
            print("FATAL ERROR:The demo requires Depth camera with Color sensor and Depth sensor")
            exit(0)

        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)#配置彩色流的参数
        config.enable_stream(rs.stream.depth, rs.format.z16, 30) #配置深度流的参数

        # Start streaming and get stream profile
        pipeline.start(config)
        profile = pipeline.get_active_profile()

        # depth camera intrinsics
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks（如何理解？不知道）
        pc = rs.pointcloud()
        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 2 ** self.decimate)
        colorizer = rs.colorizer() #fzz原始舍弃了这一行，跳过了一系列鼠标事件的绑定
        filters = [rs.disparity_transform(),
                   rs.spatial_filter(),
                   rs.temporal_filter(),
                   rs.disparity_transform(False)]

        cv2.namedWindow(self.WIN_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow(self.WIN_NAME, w, h)

        #舍弃了While true的一直循环，pw觉得应该加上？

        if not self.paused:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            

            depth_frame = frames.get_depth_frame()#获取深度帧
            color_frame = frames.get_color_frame()#获取彩色帧

            #使用深度帧的数据进行降采样，降低图像的分辨率，
            #是一种RealSense SDK 配置的深度数据滤波器，
            #方便后续的图像处理
            depth_frame = decimate.process(depth_frame)

            # Grab new intrinsics (may be changed by decimation)
            depth_intrinsics = rs.video_stream_profile(
                depth_frame.profile).get_intrinsics()
            w, h = depth_intrinsics.width, depth_intrinsics.height

            self.depth_image = np.asanyarray(depth_frame.get_data())
            self.color_image = np.asanyarray(color_frame.get_data())

