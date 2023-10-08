import cv2
import math
import numpy as np

class position(object):
    def __init__(self, rot=0, x=0, y=0, z=0):
        self.rot = rot
        self.x = x
        self.y = y
        self.z = z

def calposition(tobii_thread, tobii_show, thread_D435):
    frame_tobii = tobii_thread.get_frame()
    target_position = get_target_position(tobii_show.gp, frame_tobii, thread_D435)#通过上述的数据计算击打位置
    return target_position

def data_filter(data_save, data_give):
    data_save = (1-np.exp(-1.0/3.0))*data_give + np.exp(-1.0/3.0)*data_save
    return data_save

#获取目标击打位置的方法
def get_target_position(gp_data, img, thread):
    
    #对获取到的图像进行处理
    gray_img_tobii = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)#BGR(蓝-绿-红)颜色格式到灰度的
                                                          #颜色空间(wei)
    gray_img_D435 = cv2.cvtColor(thread.get_color_frame(), 
                                 cv2.COLOR_BGR2GRAY)
    
    #灰度转换
    size_tobii = gray_img_tobii.shape[::-1]#灰度图像的的(height,width)位置互换？(wei)
    size_D435 = gray_img_D435.shape[::-1]  #灰度图像的的(height,width)位置互换？(wei)
    gpx = gp_data[0]
    gpy = gp_data[1]

    gaze_pixel = np.array([[gpx*size_tobii[0]], 
                           [gpy*size_tobii[1]], [1]])#创建注释像素平面？(wei)
    
    qrCodeDetector = cv2.QRCodeDetector()#创建openCV创建二维码识别器(wei)
    #对图像进行二维码识别并且返回TF值返回在retval中，坐标值储存再points中，(wei)
    retval_tobii, points_tobii = qrCodeDetector.detect(gray_img_tobii)
    retval_D435, points_D435 = qrCodeDetector.detect(gray_img_tobii)#points_d435没用过,
                                                                   #应该是fzz打错了,已改为D(wei)
    
    #二维码的定位(wei)
    if(not retval_tobii):#如果第一次识别二维码不成功,采用高斯模糊来减少图像的噪声来增强图像
        img1 = cv2.GaussianBlur(gray_img_tobii,(1,1),2,2,cv2.BORDER_DEFAULT)
        retval_tobii, points_tobii = qrCodeDetector.detect(img1)
        if(not retval_tobii):#如果第二次识别二维码不成功,采用Otsu阈值法处理来binarize图像
            retval_tobii,img1 = cv2.threshold(img1,0,255,cv2.THRESH_OTSU)
            retval_tobii, points_tobii = qrCodeDetector.detect(img1)
            if(not retval_tobii):#如果第三次不成功,采用形态学方法对二值图像进行处理
                element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
                img1 = cv2.morphologyEx(img1,cv2.MORPH_OPEN,element)
                img1 = cv2.morphologyEx(img1,cv2.MORPH_CLOSE,element)
                retval_tobii, points_tobii = qrCodeDetector.detect(img1)
                if(not retval_tobii):#报错,无法识别QRcode
                    print("can not find QR code in tobii")

    if(not retval_D435):#一样套路(wei)
        img1 = cv2.GaussianBlur(gray_img_tobii,(1,1),2,2,cv2.BORDER_DEFAULT)
        retval_D435, points_D435 = qrCodeDetector.detect(img1)
        if(not retval_D435):
            retval_D435,img1 = cv2.threshold(img1,0,255,cv2.THRESH_OTSU)
            retval_D435, points_D435 = qrCodeDetector.detect(img1)
            if(not retval_D435):
                element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
                img1 = cv2.morphologyEx(img1,cv2.MORPH_OPEN,element)
                img1 = cv2.morphologyEx(img1,cv2.MORPH_CLOSE,element)
                retval_D435, points_D435 = qrCodeDetector.detect(img1)
                if(not retval_D435):
                    print("can not find QR code in D435")
    #相机标定,成像类型:凸透镜成像
    if(points_tobii is not None & points_D435 is not None):

        #QRcode的像素坐标
        #为什么不直接 points_tobii[0][0]?
        imagePoints_tobii = np.array([[points_tobii[0][0][0], points_tobii[0][0][1]],
                                      [points_tobii[1][0][0], points_tobii[1][0][1]],
                                      [points_tobii[2][0][0], points_tobii[2][0][1]],
                                      [points_tobii[3][0][0], points_tobii[3][0][1]]]).astype(float) #像素坐标,单位:像素
        imagePoints_D435 =  np.array([ [points_D435[0][0][0],  points_D435[0][0][1]],
                                       [points_D435[1][0][0],  points_D435[1][0][1]],
                                       [points_D435[2][0][0],  points_D435[2][0][1]],
                                       [points_D435[3][0][0],  points_D435[3][0][1]]]).astype(float) #像素坐标,单位:像素
        objectPoints = np.array([[0,0,0],[0,49,0],[49,49,0],[49,0,0]]).astype(float) #空间坐标,单位:毫米mm

        #相机的内参矩阵M(intrinsic)
        focal_length_tobii = 1.123161286201691e+03 #相机焦距
        cameraMatrix_tobii = np.array([[focal_length_tobii, 0, 9.470499505963581e+02],
                                       [0, focal_length_tobii, 5.492222351061334e+02],
                                       [0, 0, 1]]).astype(float) #相机内参矩阵
        
        focal_length_D435 = 1.123161286201691e+03 #深度相机焦距
        cameraMatrix_D435 = np.array([[focal_length_D435, 0, 9.470499505963581e+02],
                                      [0, focal_length_D435, 5.492222351061334e+02], 
                                      [0, 0, 1]]).astype(float) #深度相机内参矩阵
        distCoeffs = np.array([-0.002877288092662,0.032719266686277,0,0]).astype(float)#偏移?(wei)

       
        #计算相机空间位置，retval:结果，true/false rvec:旋转向量，方向为旋转轴，模值为旋转角，
        # tvec：平移向量 flags:求解方法，详见官方文档，SOLVEPNP_ITERATIVE此方法需要至少4对点
        retval_tobii, rvec_tobii, tvec_tobii = cv2.solvePnP(objectPoints,
                                                            imagePoints_tobii, 
                                                            cameraMatrix_tobii,
                                                            distCoeffs, 
                                                            flags=cv2.SOLVEPNP_ITERATIVE) 
        
        R_tobii = cv2.Rodrigues(rvec_tobii)[0] #利用罗德里格斯公式求旋转矩阵,输出为R，jacobian
        
        retval_D435, rvec_D435, tvec_D435 = cv2.solvePnP(objectPoints, 
                                                        imagePoints_D435,
                                                        cameraMatrix_D435,
                                                        distCoeffs,
                                                        flags=cv2.SOLVEPNP_ITERATIVE) 
        R_D435 = cv2.Rodrigues(rvec_D435)[0] #利用罗德里格斯公式求旋转矩阵,输出为R，jacobian
        
        #深度图像转成3D坐标
        point_cloud = np.zeros([76800,3])
        depth_frame = thread.get_depth_frame()
        for i in range(0,240):
            for j in range(0,320):
                point_cloud[320*i+j] = trans_2D_to_3D(R_D435, tvec_D435, cameraMatrix_D435,[i,j], depth_frame[i][j]/1000)
        
        #计算注视点
        point0 = np.transpose(R_tobii)@tvec_tobii #tobii原点相对世界坐标系的坐标
        point1 = np.linalg.inv(R_tobii)@(3000*np.linalg.inv(cameraMatrix_tobii)@gaze_pixel - tvec_tobii)#视线上的任意点 这里假设距离为3米（3000mm）
        point = find_world_point(point_cloud,point0,point1)
        return point



        #旧版，无D435
        # point = trans_2D_to_435(R_tobii, tvec_tobii, cameraMatrix_tobii, gaze_pixel)
        # point = trans_tobii_to_D435(R_tobii, tvec_tobii, cameraMatrix_tobii, gaze_pixel, R_D435, tvec_D435, cameraMatrix_D435)
        # print("world point is:\n",point.reshape(1,3))

        # target_position = position()
        # target_position.rot = math.atan2(point[1],point[0])
        # # target_position.x = math.sqrt(point[0]**2+point[1]**2)/1000
        # target_position.x = point[0]/1000
        # target_position.y = point[1]/1000
        # target_position.z = 0.0
        # return target_position
    else:
        print("falile to detect the QR code")
        return None

#已知像素点坐标和空间z平面以及相机内参矩阵，平移旋转向量求三维空间坐标
#R:旋转矩阵np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])，t平移向量np.array([[tx],[ty],[tz]])， 
# cameraMatrix:相机内参矩阵np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]), imagePoints:像素坐标np.array([[ux],[vy],[1]])，
# z:世界坐标系z值，默认为0
def trans_2D_to_3D(R, t, cameraMatrix, imagePoints, z=0):
    s = (z + (np.linalg.inv(R)@t)[2])/((np.linalg.inv(R)@np.linalg.inv(cameraMatrix)@imagePoints)[2]) #计算深度
    position = np.linalg.inv(R)@(s*np.linalg.inv(cameraMatrix)@imagePoints - t)#计算像素点对应的世界坐标系位置
    position[2] = z
    return position

#加入深度相机
def find_world_point(point_cloud, p1, p2): #求解点云数据中距离视线最近的点
    min_result = np.inf
    min_point = None

    # Iterate over each point in the point cloud
    for point in point_cloud:
        # Calculate the cross product of the point and the known point
        cross_product = np.cross(np.transpose(p1)-point, np.transpose(p2)-point)

        # Calculate the magnitude of the cross product
        result = np.linalg.norm(cross_product)/np.linalg.norm(p1-p2)

        # Check if this is the minimum result so far
        if result < min_result:
            min_result = result
            min_point = point

    # Return the point with the minimum result
    return min_point