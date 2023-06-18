#!/usr/bin/env python3

import rospy
import torch
import numpy as np
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ObjectDetection:
    def __init__(self):
        rospy.init_node('ObjectDetection', anonymous=True)
        self.flag_received = False
        self.device = 'cpu'
        self.model = self.load_model()
        self.classes = ['construction', 'forward', 'give_way', 'green', 'left', 'red', 'right', 'roundabout', 'stop', 'yellow']
        
        print(f'Device: {self.device}')

        rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.classes_publisher = rospy.Publisher('/detected_classes', String, queue_size=1)
        self.image_publisher = rospy.Publisher('/processed_image', Image, queue_size=1)  # New Image publisher
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.flag_received:
                frame = self.cv_image
                frame = cv2.resize(frame, (640, 640))
                frame_g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                start_time = time.time()
                results = self.score(frame_g)
                print('latency', time.time() - start_time)
                print(results)
                detected_classes, frame = self.plot_boxes(results, frame)
                
                self.classes_publisher.publish(",".join(detected_classes))
                self.image_publisher.publish(self.cv2_to_ros(frame))  # Publish processed image
                rate.sleep()

    def load_model(self):
        model_path = '/home/puzzlebot/catkin_ws/src/best2.onnx'
        model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)
        model.to(self.device)
        return model
    
    def score(self, frame):
        frame = [frame]
        results = self.model(frame)

        labels = results.xyxyn[0][:, -1].cpu().numpy()
        cord = results.xyxyn[0][:, :-1].cpu().numpy()
        
        print(labels)

        return labels, cord

    def class_to_label(self, x):
        class_names = ['construction', 'forward', 'give_way', 'green', 'left', 'red', 'right', 'roundabout', 'stop', 'yellow']
        return class_names[int(x)]
    
    def plot_boxes(self, results, frame):
        labels, cord = results
        n = len(labels)

        x_shape, y_shape = frame.shape[1], frame.shape[0]
 
        min_area = 300
        detected_classes = []

        for i in range(n):
            row = cord[i]
            if row[4] > 0.5: 
                x1, y1 = int(row[0] * x_shape), int(row[1] * y_shape)
                x2, y2 = int(row[2] * x_shape), int(row[3] * y_shape)
                area = (x2 - x1) * (y2 - y1)

                if area > min_area: 
                    object_class = self.class_to_label(labels[i])
                    if object_class in ["left", "right"]:
                         #Cropping the region of interest (ROI)
                         roi = frame[y1:y2, x1:x2]
                         roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                         _, binary_roi = cv2.threshold(roi, 127, 255, cv2.THRESH_BINARY)
                    
                         # Divide the image into four segments
                         h, w = binary_roi.shape
                         upper_left = binary_roi[:h//2, :w//2]
                         upper_right = binary_roi[:h//2, w//2:]
                    
                         # Counting white pixels in each upper segment
                         upper_left_white = np.count_nonzero(upper_left == 255)
                         upper_right_white = np.count_nonzero(upper_right == 255)
                    
                         # Verifying left or right
                         object_class = "right" if upper_right_white > upper_left_white else "left"
                
                    detected_classes.append(object_class)
                    print("Detected Object Class:", object_class)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw rectangle on frame

        if not detected_classes:
            detected_classes.append('no')

        return detected_classes, frame  # Return frame as well

    def cv2_to_ros(self, cv_image):
        image_message = Image()
        image_message.height = cv_image.shape[0]
        image_message.width = cv_image.shape[1]
        image_message.encoding = 'bgr8'
        image_message.is_bigendian = False
        image_message.step = cv_image.shape[1] * 3  # Full row length in bytes
        image_message.data = cv_image.ravel().tolist()
        return image_message

    def ros_to_cv2(self, img_msg):
        img_array = np.frombuffer(img_msg.data, dtype=np.uint8)
        img = np.reshape(img_array, (img_msg.height, img_msg.width, -1))
        return img

    def image_callback(self, msg):
        try:
            self.cv_image = self.ros_to_cv2(msg)
            self.flag_received = True
        except Exception as e:
            rospy.logerr("Error converting ROS Image to OpenCV image: {}".format(str(e)))
            self.flag_received = False


if __name__ == '__main__':
    ObjectDetection()

