#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point
from matplotlib.patches import Rectangle
import math

lidar_data = None
center_x, center_y = 0, 0
threshold_distance = 0.5
human_position = [0, 0, 0]


def lidar_callback(data):
    global lidar_data
    lidar_data = data

def human_position_callback(data):
    global human_position
    human_position = [data.x, data.y,data.z]

def tinh_goc_tap(O, A):
    # Tính độ dài các cạnh của tam giác vuông OA và Ox
    canh_ngang = A[0] - O[0]  # Độ dài theo trục x
    canh_doc = A[1] - O[1]    # Độ dài theo trục y

    # Tính góc tạp bởi sử dụng hàm atan2
    goc_tap = math.atan2(canh_doc, canh_ngang)
    
    # Chuyển đổi radian sang độ
    goc_tap_deg = math.degrees(goc_tap)

    return goc_tap_deg

def cung_dau(a, b):
    if (a * b) >= 0:
        return 1
    else :
        return 0


def plot_lidar_data():
    # Initialize cumulative forces
    Fxr, Fyr,Fxg,Fyg = 0, 0, 0, 0
    plt.clf()
    ranges = np.array(lidar_data.ranges)
    angle_min = lidar_data.angle_min
    angle_max = lidar_data.angle_max
    angle_increment = lidar_data.angle_increment
    angles = np.linspace(angle_min, angle_max, len(ranges))

    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)

    xs -= center_x
    ys -= center_y
    # Obstacle points
    

    ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)

    distances_from_center = np.sqrt(xs**2 + ys**2)
    close_to_center = distances_from_center < threshold_distance

    

    plt.scatter(xs[~close_to_center], ys[~close_to_center], s=2, c='blue', alpha=0.5)
    plt.scatter(0, 0, s=50, c='red', marker='x', label='Center')
    plt.scatter(human_position[0], human_position[1], s=100, c='green', marker='o', label='Human')
    plt.plot([-3, 3], [0, 0], linestyle='--', color='gray')
    plt.plot([0, 0], [-3, 3], linestyle='--', color='gray')

    # Add a square
    height,width = 0.6,0.4 # Define the side length of the square
    square = Rectangle((-height/2, -width /2), height,width, edgecolor='orange', facecolor='none')
    plt.gca().add_patch(square)

    # Add a square
    height,width = 2,2 # Define the side length of the square
    square = Rectangle((-height/2, -width /2), height,width, edgecolor='orange', facecolor='none')
    plt.gca().add_patch(square)

    # Identify points within the square
    
    points_inside_square = np.logical_and(np.abs(xs) < height / 2, np.abs(ys) < width / 2)

    # Lọc các điểm có xs > 0
    filtered_points = xs[points_inside_square] > 0

    # Tạo mảng chứa các điểm xs > 0
    xs_positive = xs[points_inside_square][filtered_points]
    ys_positive=ys[points_inside_square][filtered_points]

    # Plot các điểm trong hình vuông
    plt.scatter(xs[points_inside_square], ys[points_inside_square], s=5, c='green', alpha=0.8)

    # Plot các điểm có xs > 0
    plt.scatter(xs_positive, ys_positive, s=5, c='black', alpha=0.8)

    stop = 1

    for i in range(len(xs_positive)):

        R = np.sqrt((xs_positive[i] - center_x)**2 + (ys_positive[i] - center_y)**2)
        Fx = -0.05 * (xs_positive[i] - center_x) / (R**3)
        Fy = -0.05 * (ys_positive[i] - center_y) / (R**3)
        Rg =  np.sqrt((human_position[0] - center_x)**2 + (human_position[1] - center_y)**2)
        Fxg = 10*(human_position[0] - center_x) / (Rg)
        Fyg = 10*(human_position[1] - center_y) / (Rg)

        Fxr += Fx
        Fyr += Fy
        if Rg < 0.5 :
            stop = 0
#        print(f"Góc của điểm {i,xs_positive[i], ys_positive[i]} ")

        #plt.quiver(center_x, center_y, Fx , Fy, angles='xy', scale_units='xy', scale=1, color='green', alpha=0.5)
    plt.quiver(center_x, center_y, Fxr+Fxg, Fyr+Fyg, angles='xy', scale_units='xy', scale=1, color='red', alpha=0.5)
    angular = tinh_goc_tap((center_x, center_y),(Fxr+Fxg, Fyr+Fyg))
    #print(f"Góc của điểm {Fxr+Fxg, Fyr+Fyg} là: {angular} độ")
    
    lower_limit = -180
    upper_limit = 180
    limited_angular_z = max(min(angular, upper_limit), lower_limit)
    if math.isnan(limited_angular_z):
        limited_angular_z = 0
    if stop == 1:
        twist_msg.angular.z = limited_angular_z
        twist_msg.linear.x=5000
    if stop == 0:
        twist_msg.angular.z = 0
        twist_msg.linear.x=0
    print(twist_msg)
    #distances_from_center tìm điểm gần ox nhất

            


    #plt.scatter(xs[close_to_center], ys[close_to_center], s=5, c='red', alpha=0.8)  



    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LiDAR Scan Data with Objects Close to Center Highlighted in Red')
    plt.axis('equal')
    plt.legend()
    plt.pause(0.00001)
    plt.draw()

def control_robot():
    rospy.init_node('lidar_listener', anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.Subscriber("/human_position_topic", Point, human_position_callback)
    
    plt.ion()
    plt.show()

    while not rospy.is_shutdown():
        if lidar_data is not None:
            plot_lidar_data()
            pub.publish(twist_msg)

            

if __name__ == '__main__':
    twist_msg = Twist()
    control_robot()
