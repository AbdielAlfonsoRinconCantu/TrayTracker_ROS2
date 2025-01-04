################################################################
# Camera Subscriber Node
# Author: Aleksandar Haber
# Date: August 2024
# License: This code is the ownership of Aleksandar Haber
# send an email to ml.mecheng@gmail.com to inform yourself
# about the license 
# for commercial projects
################################################################

# Import the libraries
# this is OpenCV
import cv2

# these are ROS2 package modules and libraries
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

# the argument "Node" means that the PublisherNodeClass inherits (or is a child of)
# the class called Node. The Node class is a standard ROS2 class

class SubscriberNodeClass(Node):

    # constructor
    def __init__(self):
        
        # this function is used to initialize the attributes of the parent class
        # super().__init__('subsciber_node')
        super().__init__('tray_image_subsciber')

        # CvBridge is used to convert OpeCV images to ROS2 messages that can be sent throught the topics
        self.BridgeObject = CvBridge()

        # name of the topic used to transfer the camera images
        # this topic name should match the topic name in the subscriber node
        # self.topicNameFrames='topic_camera_image'
        self.topicNameFrames='/track_tray_image'

        # the queue size for messages
        self.queueSize=20

        # here, the function "self.create_publisher" creates the publisher that
        # publishes the messages of the type Image, over the topic self.topicNameFrames
        # and with self.queueSize
        
        self.subscription = self.create_subscription(Image,self.topicNameFrames,self.listener_callbackFunction,self.queueSize)
        self.subscription # This is used to prevent unused variable warning



    # this is the callback function that displays the received image
    def listener_callbackFunction(self, imageMessage):
        # Display the message on the console
        self.get_logger().info('The image frame is received')
        
        # convert ROS2 image message to openCV image
        openCVImage = self.BridgeObject.imgmsg_to_cv2(imageMessage)
        
        # Show the image on the screen
        cv2.imshow("/track_tray_image", openCVImage)
        cv2.waitKey(1)

# this is the main function and this is the entry point of our code
def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)

    # create the subscriber object
    subscriberNode = SubscriberNodeClass()

    # here we spin, and the callback timer function is called recursively
    rclpy.spin(subscriberNode)

    # destroy
    subscriberNode.destroy_node()

    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
