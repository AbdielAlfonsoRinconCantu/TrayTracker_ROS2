#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int32MultiArray

from ultralytics import solutions
from ultralytics.solutions import object_counter
from ultralytics import YOLO

class yolo_tracking(Node):
    def __init__(self):
        super().__init__("tray_tracker") # not even the most accurate name, but sounds cool

        self.image_pub = self.create_publisher(Image, '/track_tray_image', 1)
        self.centroid_pub = self.create_publisher(Int32MultiArray, '/coordinates_pennies', 10)

        self.image_sub = self.create_subscription(
            Image, '/image', self.image_callback, 1)

        self.bridge = CvBridge()

        self.image_pub_2 = self.create_publisher(
            Image, '/count_penny_image', 1)

        self.region_points = [(270, 480), (370, 480), (370, 0), (270, 0)]

        self.counter = solutions.ObjectCounter(
            show=False,
            region=self.region_points,
            model="best_s.pt",
            classes=[1],
            draw_tracks=True,
            show_in=False,
            show_out=False
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            previous_count = self.counter.in_count

            processed_image = self.counter.count(cv_image)

            if self.counter.in_count > previous_count:
                self.process_penny_count(cv_image)
            
            try:
                ros_image_2 = self.bridge.cv2_to_imgmsg(self.processed_image_2, "bgr8")
                self.image_pub_2.publish(ros_image_2)
            except:
                print("No penny count image to publish yet")

            ros_image = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            self.image_pub.publish(ros_image)

        except CvBridgeError as e:
            print(e)

    def process_penny_count(self, cv_image):
        try:
            model = YOLO('penny.pt')
            results = model(cv_image, conf=0.94)
            names = model.names
            penny_id = list(names)[list(names.values()).index('penny')]
            penny_count = results[0].boxes.cls.tolist().count(penny_id)
            print(f"Penny count: {penny_count}")

            # Initialize coordinates list with penny count as first pair
            coordinates = [0, penny_count]  # First pair (0, penny_count)

            # Get centroids for all detections
            boxes = results[0].boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()  # get box coordinates
                centroid_x = round((x1 + x2) / 2)
                centroid_y = round((y1 + y2) / 2)
                coordinates.extend([centroid_x, centroid_y])
                
            # Create and publish centroid message
            centroid_msg = Int32MultiArray()
            centroid_msg.data = coordinates
            self.centroid_pub.publish(centroid_msg)

            self.processed_image_2 = results[0].plot(conf=False, labels=False)

        except CvBridgeError as e:
            print(e)



def main(args=None):
    rclpy.init(args=args)

    node = yolo_tracking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
