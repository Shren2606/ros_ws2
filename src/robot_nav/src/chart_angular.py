#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

class PlotNode:
    def __init__(self):
        rospy.init_node('plot_node', anonymous=True)

        # Khởi tạo danh sách giá trị và thời gian
        self.time_data = []
        self.value_data_z = []

        # Tạo subscriber để đăng ký nhận giá trị từ topic
        rospy.Subscriber('human_position_topic', Point, self.callback)

    def callback(self, data):
        # Callback này được gọi mỗi khi nhận được giá trị từ topic
        rospy.loginfo("Received value: x=%f, y=%f, z=%f", data.x, data.y, data.z)

        # Lưu giá trị và thời gian
        self.time_data.append(rospy.Time.now().to_sec())
        self.value_data_z.append(data.z)

        # Vẽ biểu đồ
        self.plot_graph()

    def plot_graph(self):
        # Vẽ biểu đồ chỉ với giá trị 'z'
        plt.plot(self.time_data, self.value_data_z)
        plt.title('Biểu đồ giá trị Z theo thời gian')
        plt.xlabel('Thời gian (s)')
        plt.ylabel('Giá trị Z')
        plt.ylim(-90, 90)  # Đặt giới hạn trục y từ -90 đến 90
        plt.legend()
        plt.grid(True)
        plt.pause(0.1)  # Cần pause để biểu đồ được cập nhật

if __name__ == '__main__':
    try:
        plot_node = PlotNode()
        plt.ion()  # Bật chế độ tương tác cho biểu đồ
        plt.show()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
