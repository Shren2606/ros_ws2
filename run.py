import tkinter as tk
import subprocess
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
from PIL import Image, ImageTk


class ImageSubscriber:
    def __init__(self, topic_name):
        self.bridge = CvBridge()
        self.last_image = None
        self.image_sub = rospy.Subscriber(topic_name, ROSImage, self.callback)
        
    def callback(self, ros_image):
        # Chuyển đổi ROS Image thành OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        # Chuyển đổi thành định dạng RGB để hiển thị với PIL
        self.last_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Cập nhật hình ảnh trên giao diện người dùng
        update_frame(self)


def run_command_and_display():
    # Chạy lệnh roslaunch
    command = "roslaunch robot_nav run_multiple.launch"
    subprocess.Popen(command, shell=True)

    rospy.rospin()
    
def display():
    # Khởi tạo ROS node
    rospy.init_node('image_viewer')
    # Tạo subscriber cho topic hình ảnh
    image_subscriber = ImageSubscriber("/camera/image")
    update_frame(image_subscriber)


def update_frame(image_subscriber):
    # Lấy hình ảnh từ subscriber và hiển thị trên label
    frame = image_subscriber.last_image
    
    
    if frame is not None:
        img = Image.fromarray(frame)
    else:
        # Nếu không có hình ảnh, hiển thị một hình ảnh màu xám
        img = Image.new('RGB', (640, 480), color='gray')
    img = ImageTk.PhotoImage(image=img)
    label.config(image=img)
    label.image = img  # Cập nhật image attribute của label

    # Lặp lại việc cập nhật frame sau một khoảng thời gian
    #window.after(1, update_frame, image_subscriber)  # Đổi từ 1ms thành 100ms hoặc số lớn hơn nếu cần
def run_rosserial():
    command = "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600"
    subprocess.Popen(command, shell=True)

# Tạo cửa sổ
window = tk.Tk()
window.title("Chạy Lệnh Robot và Hiển Thị Hình Ảnh từ ROS Topic")

# Tạo label để hiển thị hình ảnh từ ROS topic
label = tk.Label(window)
label.pack()

"""img = Image.new('RGB', (640, 480), color='gray')
img = ImageTk.PhotoImage(image=img)
label.config(image=img)
label.image = img"""

# Tạo nút nhấn để chạy lệnh roslaunch và hiển thị hình ảnh
button_run_display = tk.Button(window, text="Khoi dong", command=run_command_and_display)
button_run_display.pack(side=tk.LEFT, padx=20, pady=10)

button_run_serial = tk.Button(window, text="display", command=display)
button_run_serial.pack(side=tk.LEFT, padx=20, pady=10)

# Tạo nút nhấn để chạy lệnh rosserial
button_run_serial = tk.Button(window, text="Run", command=run_rosserial)
button_run_serial.pack(side=tk.LEFT, padx=20, pady=10)

# Bắt đầu vòng lặp sự kiện của Tkinter
window.mainloop()
