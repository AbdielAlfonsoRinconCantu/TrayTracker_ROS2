################################################################
# Camera Publisher Node
# Author: Aleksandar Haber
# Date: August 2024
# License: This code is the ownership of Aleksandar Haber
# send an email to ml.mecheng@gmail.com to inform yourself
# about the license for using this code in commercial projects
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
class PublisherNodeClass(Node):

    # constructor
    def __init__(self):
        
        # this function is used to initialize the attributes of the parent class
        super().__init__('publisher_node')

        # here, we create an instance of the OpenCV VideoCapture object
        # this is the camera device number - you need to properly adjust this number
        # depending on the camera device number assignmed by the Linux system
        self.cameraDeviceNumber=0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

        # CvBridge is used to convert OpenCV images to ROS2 messages that can be sent throught the topics
        self.bridgeObject = CvBridge()

        # name of the topic used to transfer the camera images
        # this topic name should match the topic name in the subscriber node
        self.topicNameFrames='topic_camera_image'

        # the queue size for messages
        self.queueSize=20

        # here, the function "self.create_publisher" creates the publisher that
        # publishes the messages of the type Image, over the topic self.topicNameFrames
        # and with self.queueSize
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

        # communication period in seconds
        self.periodCommunication = 0.02

        # Create the timer that calls the function self.timer_callback every self.periodCommunication seconds
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)

        # this is the counter tracking how many images are published
        self.i = 0

    # this is the callback function that is called every self.periodCommunication seconds
    def timer_callbackFunction(self):
        # here we read the frame by using the camera
        success, frame = self.camera.read()
        # resize the image
        frame = cv2.resize(frame, (426,240), interpolation=cv2.INTER_CUBIC)
        
        # if we are able to read the frame
        if success == True:
            # here, we convert OpenCV frame to
            ROS2ImageMessage=self.bridgeObject.cv2_to_imgmsg(frame) # here
            # publish the image
            self.publisher.publish(ROS2ImageMessage)

            # Use the logger to display the message on the screen
            self.get_logger().info('Publishing image number %d' % self.i)
            # update the image counter
            self.i += 1

# this is the main function and this is the entry point of our code
def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)

    # create the publisher object
    publisherObject = PublisherNodeClass()

    # here we spin, and the callback timer function is called recursively
    rclpy.spin(publisherObject)

    # destroy
    publisherObject.destroy_node()

    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
