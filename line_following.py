#!/usr/bin/env python

import rospy 
import numpy as np

from sensor_msgs.msg import Image 

from cv_bridge import CvBridge, CvBridgeError 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import cv2 

from std_msgs.msg import String



class line_detector(object): 


    def _init_(self): 

        rospy.on_shutdown(self.cleanup) 
        
        
        rospy.Subscriber("/video_source/raw",Image,self.camera_callback) 
        rospy.Subscriber("/detected_classes",String,self.classes_callback)
        #rospy.Subscriber("/camera/image_raw",Image,self.camera_callback) 

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.image_publisher = rospy.Publisher('/cross_image', Image, queue_size=1)
        robot_vel = Twist() #The robot's velocity 
        

        robot_vel.linear.x=0
        robot_vel.angular.z=0
        self.average_value = 0
        self.construction_detected = False

        self.class_received = False
        self.bridge_object = CvBridge() 

        ###########CONTROL#############
        self.multiplicador = 1

        self.e=[0.0,0.0,0.0]
        self.u=[0.0,0.0]

        self.e_l=[0.0,0.0,0.0]
        self.u_l=[0.0,0.0]
        self.Ts=0.002


        self.kp=0.006
        self.ki=0.000
        self.kd=0.00006
        
        self.detected_classes = []

        self.multiplicador = 1

        self.state = "line_follow" # initialize the state to line following
        self.previous_state = "line_follow"
        self.anti_previous_state = "line_follow"
        self.cross_sum = 3000000
        self.lim_cross = 900000

        r = rospy.Rate(20) #10Hz  
              
        

        ##################################

        self.image_received = False #Flag to indicate that we have already received an image 

        #self.detected_classes = []

        self.current_time = 0
        self.cross_sum = 0

        while not rospy.is_shutdown():  
            self.update()
            if self.image_received : 
                self.update()
                #print(self.cross_sum)
                robot_vel = Twist() 

                # update state based on traffic signs
                if self.detected_classes:
                    self.update()
                    sign = self.detected_classes[0]
                   

                    if sign == "stop" and self.anti_previous_state != "stop":
                        self.state = "stop"
                    elif sign == "give_way":
                        self.state = "give_way"
                    elif sign == "construction" and self.previous_state != "construction" and self.construction_detected == False:
                        self.state = "construction"
                        self.construction_detected = True
                        self.start_time_c = rospy.Time.now()
                    elif sign == "forward":
                        self.state = "forward_wait_cross"
                    elif sign == "right":
                        self.state = "right_wait_cross"
                    elif sign == "left":
                        self.state = "left_wait_cross"

                

                if self.state == "line_follow":
                    print("Estado: ",self.state)
                    print("Previous_State: ",self.previous_state)
                    print("Anti Previous_State: ",self.anti_previous_state)
                    self.update()
                    robot_vel.linear.x=0.07*self.multiplicador

                    robot_vel.angular.z= self.u[0]*self.multiplicador

                    if self.cross_sum < self.lim_cross:
                        self.start_time = rospy.Time.now()
                        if self.state == "forward_wait_cross":
                            self.update_state()
                            self.state = "forward_cross"
                        elif self.state == "right_wait_cross":
                            self.update_state()
                            self.state = "right_cross"
                        elif self.state == "left_wait_cross":
                            self.update_state()
                            self.state = "left_cross"
                        else:
                            self.update_state()
                            self.state = "forward_cross"

                    

                elif self.state == "give_way":
                    print("Estado: ",self.state)
                    print("Previous_State: ",self.previous_state)
                    print("Anti Previous_State: ",self.anti_previous_state)

                    current_time = rospy.Time.now()
                    if (current_time - self.start_time).to_sec() <= 5:
                        robot_vel.linear.x=0.035
                        robot_vel.angular.z= self.u[0]
                    else:
                        self.anti_previous_state = self.previous_state
                        self.previous_state = self.state
                        self.update_state()
                        self.state = "line_follow"
                        

                elif self.state == "construction":
                    print("Estado: ",self.state)
                    print("Previous_State: ",self.previous_state)
                    print("Anti Previous_State: ",self.anti_previous_state)

                    current_time_c = rospy.Time.now()
                    if (current_time_c - self.start_time_c).to_sec() <= 2:
                        print("Disminuyendo velocidad")
                        robot_vel.linear.x=0.035
                        robot_vel.angular.z= self.u[0]
                    else:
                        self.update_state()
                        self.construction_detected = False
                        self.state = "line_follow"
                        
                elif self.state == "stop":
                    print(self.state)
                    robot_vel.linear.x = 0
                    robot_vel.angular.z = 0

                elif self.state in ["forward_wait_cross", "right_wait_cross", "left_wait_cross"]:
                    print("Estado: ",self.state)
                    print("Previous_State: ",self.previous_state)
                    print("Anti Previous_State: ",self.anti_previous_state)

                    self.update()
                    robot_vel.linear.x=0.07*self.multiplicador
                    robot_vel.angular.z= self.u[0]*self.multiplicador
                    if self.cross_sum < self.lim_cross:
                        self.start_time = rospy.Time.now()
                        if self.state == "forward_wait_cross":
                            self.update_state()
                            self.state = "forward_cross"
                        elif self.state == "right_wait_cross":
                            self.update_state()
                            self.state = "right_cross"
                        elif self.state == "left_wait_cross":
                            self.update_state()
                            self.state = "left_cross"
                    
                    

                elif self.state == "right_cross":
                    print("Estado: ",self.state)
                    print("Previous_State: ",self.previous_state)
                    print("Anti Previous_State: ",self.anti_previous_state)

                    time_passed = (rospy.Time.now() - self.start_time).to_sec()
                    if time_passed <= 1:  # Advance 10cm
                        robot_vel.linear.x=0.3
                        robot_vel.angular.z=0.01
                    elif time_passed <= 3:  # Rotate 90 degrees
                        robot_vel.linear.x=0
                        robot_vel.angular.z=-0.8
                    elif time_passed <= 3.25:  # Advance 10cm
                        robot_vel.linear.x=0.25
                        robot_vel.angular.z=0
                    else:
                        self.multiplicador = 1
                        self.update_state()
                        self.state = "line_follow"

                    

                elif self.state == "left_cross":
                    print("Estado: ",self.state)
                    print("Previous_State: ",self.previous_state)
                    print("Anti Previous_State: ",self.anti_previous_state)

                    time_passed = (rospy.Time.now() - self.start_time).to_sec()
                    if time_passed <= 1:  # Advance 10cm
                        robot_vel.linear.x=0.3
                        robot_vel.angular.z=-0.01
                    elif time_passed <= 3:  # Rotate 90 degrees
                        robot_vel.linear.x=0
                        robot_vel.angular.z=0.8
                    elif time_passed <= 3.25:  # Advance 10cm
                        robot_vel.linear.x=0.25
                        robot_vel.angular.z=0
                    else:
                        self.multiplicador = 1
                        self.update_state()
                        self.state = "line_follow"

              
                elif self.state == "forward_cross":
                    print("Estado: ",self.state)
                    print("Previous_State: ",self.previous_state)
                    print("Anti Previous_State: ",self.anti_previous_state)

                    time_passed = (rospy.Time.now() - self.start_time).to_sec()
                    if time_passed <= 0.5:  # Advance 10cm
                        robot_vel.linear.x=0
                        robot_vel.angular.z=0
                    elif time_passed <= 3:  # Rotate 90 degrees
                        robot_vel.linear.x=0.3
                        robot_vel.angular.z=-0.0
                    else:
                        self.multiplicador = 1
                        self.update_state()
                        self.state = "line_follow"
                    

                    

                

                self.cmd_vel_pub.publish(robot_vel)
                r.sleep()  
 
    def update_state(self):
        self.anti_previous_state = self.previous_state
        self.previous_state = self.state


    def update(self):
        if self.image_received:
            vertical_sum = np.sum(self.gray, axis=0)
            indices = np.argsort(vertical_sum)
            n = 5
            smallest_values = indices[:n]
            self.average_value = np.mean(smallest_values)
            self.average_value = int(self.average_value)
            ###########################################

            sum_cross = np.sum(self.gray2, axis=1)
            self.cross_sum = np.sum(sum_cross)
            
            ###########CONTROL#############
            K1=self.kp+self.Ts*self.ki+self.kd/self.Ts
            K2=-self.kp-2.0*self.kd/self.Ts
            K3=self.kd/self.Ts

            #####CONTROLLER######3
            self.e[0]=50-self.average_value
            self.u[0]=K1*self.e[0]+K2*self.e[1]+K3*self.e[2]+self.u[1]
            #self.u[0]=self.limit_speed(self.u[0],1.5)
            self.u[0]=self.u[0]
            self.e[2]=self.e[1]
            self.e[1]=self.e[0]
        
            self.u[1]=self.u[0]
            print("Error: ",self.e[0])
            print("Control: ",self.u[0])

           


    def camera_callback(self,data): 

        try: 

            # We select bgr8 because its the OpenCV encoding by default 

            image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") 
            self.cv_image = cv2.resize(image,(400,400))
         
            self.line_image = self.cv_image[390:400,100:300]
            self.cross_image = self.cv_image[245:275,30:370]
            self.gray2 =cv2.cvtColor(self.cross_image, cv2.COLOR_BGR2GRAY)
            self.gray = cv2.cvtColor(self.line_image, cv2.COLOR_BGR2GRAY)
            self.image_received=True
           

        except CvBridgeError as e: 

            print(e) 

    
    def classes_callback(self,msg):
        detected_classes_str = msg.data  # Obtener la cadena de clases detectadas
        self.detected_classes = detected_classes_str.split(',') 

        #Detectar banderas#
        if 'green' in self.detected_classes:
            self.multiplicador = 1
        elif 'yellow' in self.detected_classes:
            self.multiplicador = 0.5
        elif 'red' in self.detected_classes:
            self.multiplicador = 0
        
        # print(self.detected_classes)
        # print(self.detected_classes)
        self.class_received = True
    

    def limit_speed(self, speed, limit):
        if abs(speed) > limit:
            return np.sign(speed) * limit
        return speed
    
    def cv2_to_ros(self, cv_image):
        image_message = Image()
        image_message.height = cv_image.shape[0]
        image_message.width = cv_image.shape[1]
        image_message.encoding = 'bgr8'
        image_message.is_bigendian = False
        image_message.step = cv_image.shape[1] * 3  # Full row length in bytes
        image_message.data = cv_image.ravel().tolist()
        return image_message

    def cleanup(self):  
    
        vel_msg = Twist() 
        vel_msg.linear.x=0
        vel_msg.angular.z=0

        self.cmd_vel_pub.publish(vel_msg)


if _name_ == '_main_': 

    rospy.init_node('line_detector', anonymous=True) 

    line_detector()
