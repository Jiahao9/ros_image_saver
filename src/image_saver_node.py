#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import pygame
import time

class ImageSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_saver', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Subscribe to the /rgb/image_raw topic
        self.image_sub = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        
        # Initialize last capture time
        self.last_capture_time = time.time()    

        # Initialize pygame for playing sounds
        pygame.mixer.init()

    def image_callback(self, data):
        try:
            # Convert the received image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return
        
       # Check if 5 seconds have elapsed since the last capture
        current_time = time.time()
        if current_time - self.last_capture_time >= 5:
            self.last_capture_time = current_time
            self.save_image(cv_image)
            self.play_bump_sound()

    def save_image(self, image):
        img_path = "/home/rg-dell-04/catkin_ws/src/image_saver/images"
        if not os.path.exists(img_path):
            os.makedirs(img_path)

        # Get the current time as the image index
        index = len(os.listdir(img_path)) + 1

        img_name = os.path.join(img_path, 'image_{}.jpg'.format(index))
        cv2.imwrite(img_name, image)
        rospy.loginfo("Saved image: {}".format(img_name))

    def play_bump_sound(self):
        # Load and play the bump sound
        pygame.mixer.music.load("/home/rg-dell-04/catkin_ws/src/image_saver/bump_sound.wav")
        pygame.mixer.music.play()

def main():
    # Create an instance of the ImageSaver class
    image_saver = ImageSaver()
    try:
        # Spin to keep the node alive
        rospy.spin()
    except KeyboardInterrupt:
        # Log a message when shutting down the node
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
