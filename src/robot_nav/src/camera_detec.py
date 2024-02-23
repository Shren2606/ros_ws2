#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
from geometry_msgs.msg import Twist,Point

import math

import random


counter = 0
status = False  # Sử dụng cờ để theo dõi việc phát hiện

# Thử nghiệm với một số giá trị đầu vào
x_hat_prev = 0  # Trạng thái ước lượng trước đó
sigma_x_hat_prev = 1  # Độ lệch chuẩn của trạng thái ước lượng trước đó

z = 0  # Đo lường tại thời điểm hiện tại
sigma_v = 0.01  # Độ lệch chuẩn của nhiễu đo lường
sigma_w = 0.1  # Độ lệch chuẩn của nhiễu quá trình

def detect_person(image):
    global counter # Khởi tạo counter
    global status
    try:   
        if counter <= 100:
            counter += 1
            rospy.loginfo(f"đọc video từ pulisher: {counter}")
        else:
            (H, W) = image.shape[:2]
            blob = cv2.dnn.blobFromImage(image, 0.007843, (W, H), 127.5)
            detector.setInput(blob)
            person_detections = detector.forward()
            
            for i in range(person_detections.shape[2]):
                confidence = person_detections[0, 0, i, 2]
                if confidence > 0.8:
                    idx = int(person_detections[0, 0, i, 1])
                    if CLASSES[idx] != "person":
                        rospy.logwarn("No person found")
                        continue
                    rospy.logwarn("Found someone")
                    person_box = person_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                    (startX, startY, endX, endY) = person_box.astype("int")
                    saved_frame = image[startY:endY, startX:endX]
                    
                    cv2.imwrite(f'/home/nhathai/ros_ws/data/frame_template.jpg', saved_frame)
                    status = True
    except Exception as e:
        print(e)

def kalman_filter(x_hat_prev, sigma_x_hat_prev, z, sigma_v, sigma_w):

    # Bước Dự Báo
    
    # Cập nhật độ lệch chuẩn của trạng thái dự đoán
    A = 1/(sigma_x_hat_prev + sigma_v) + 1/sigma_w
    sigma_x_hat_pred = 1/A
    
    # Hệ số Kalman
    K = sigma_x_hat_pred /sigma_w
    
    # Bước Dự Báo - Cập nhật trạng thái dự đoán và độ lệch chuẩn của nó
    x_hat = x_hat_prev + K * (z - x_hat_prev)

    x_hat_prev = x_hat 
    sigma_x_hat_prev = sigma_x_hat_pred
    return x_hat, x_hat_prev, sigma_x_hat_prev

def follow(image,depth_frame):
    global x_hat_prev
    global sigma_x_hat_prev
    try :
        
        pub_point = rospy.Publisher("/human_position_topic",Point,queue_size=1)
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
       
        template = cv2.imread('/home/thien/ros_ws/data/frame_template.jpg')
        #template = imutils.resize(template, width=300)
        

        #image = cv2.rotate(image,cv2.ROTATE_90_COUNTERCLOCKWISE)# xoay ảnh 90*
        #image = imutils.resize(image, width=300,height=300)# resize ảnh

        height, width, _ = image.shape
        part_width = width // 3
        cv2.line(image, (1 * part_width, 0), (1 * part_width, height), (255, 0, 0), 2)
        cv2.line(image, (2 * part_width, 0), (2* part_width, height), (255, 0, 0), 2)

        result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc  = cv2.minMaxLoc(result)
        w, h = template.shape[1], template.shape[0]
        top_left = max_loc
        if max_val > 0.7:
            bottom_right = (top_left[0] + w, top_left[1] + h)
            tâm_x = (top_left[0] + bottom_right[0]) // 2
            tâm_y = (top_left[1] + bottom_right[1]) // 2
            setpoint = 320
            angular=(setpoint-tâm_x)*150/640
            # Vẽ hộp giới hạn xung quanh vật

            distance = depth_frame.get_distance(tâm_x, tâm_y)  - 0.6

            # Áp dụng bộ lọc Kalman
            x_hat_result, x_hat_prev, sigma_x_hat_prev = kalman_filter(x_hat_prev, sigma_x_hat_prev, distance , sigma_v, sigma_w)
            
            # Gửi thông tin vị trí của người lên topic
            # Chuyển đổi độ sang radian
            goc_radian = math.radians(angular)

            # Tính giá trị của tangent của góc
            tan_goc = math.tan(goc_radian)

                # Tính cạnh còn lại của tam giác
            canh_con_lai = (x_hat_result) * tan_goc

            
            cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
            cv2.circle(image, (tâm_x, tâm_y), 5, (255, 255, 0), -1)
            cv2.putText(image, "{:.2f}KF".format(x_hat_result ), (tâm_x, tâm_y - 20),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
            cv2.putText(image, "{:.2f}m".format(distance), (tâm_x-150, tâm_y - 20),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 0), 2)
            
            
            # Tạo message HumanPositionMsg và gán giá trị x, y
            human_position_msg = Point()
            human_position_msg.x =x_hat_result
            human_position_msg.y = canh_con_lai
            human_position_msg.z = angular

            # Publish tọa độ người lên topic
            pub_point.publish(human_position_msg)

            twist_msg = Twist()

            twist_msg.angular.z = angular
            twist_msg.linear.x=5000

            #pub.publish(twist_msg)

            # Khởi tạo publisher cho topic hình ảnh
            image_pub = rospy.Publisher('/camera/image', ROSImage, queue_size=10)
            # Khởi tạo CvBridge
            bridge = CvBridge()
            # Chuyển đổi hình ảnh từ OpenCV sang ROS Image message
            ros_image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

                # Publish ROS Image message
            image_pub.publish(ros_image_msg)

        else:
            angular = 0
        
        # Khởi tạo publisher cho topic hình ảnh
        image_pub = rospy.Publisher('/camera/image', ROSImage, queue_size=10)
        # Khởi tạo CvBridge
        bridge = CvBridge()
        # Chuyển đổi hình ảnh từ OpenCV sang ROS Image message
        ros_image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # Publish ROS Image message
        image_pub.publish(ros_image_msg)
            
        
        
        #rospy.loginfo(f'Follow person!, max_val: {max_val}')
    except Exception as e:
        print(e)
   

def realsense_node():
    rospy.init_node('realsense_node', anonymous=True)

    rate = rospy.Rate(30)  # Tần suất 30Hz

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            if status is False:  # Nếu chưa phát hiện người, tiếp tục xử lý hình ảnh
                
                detect_person(color_image)
                
            if status is True:
                follow(color_image,depth_frame)
                

            

            rate.sleep()

    finally:
        pipeline.stop()


if __name__ == '__main__':
    try:
        prototxt_path = "/home/thien/ros_ws/model/MobileNetSSD_deploy.prototxt"
        model_path = "/home/thien/ros_ws/model/MobileNetSSD_deploy.caffemodel"
        CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person"]

        detector = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
        
        
        realsense_node()
    except rospy.ROSInterruptException:
        pass
