#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image , LaserScan
import cv2
import numpy as np
import torch
from cv_bridge import CvBridge
from objectdetection_msgs.msg import ObjectDetection, ObjectDetectionList

# Load the YOLOv5 custom model with object class "Dustbin"
model_custom = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/dhaval_lad/dhaval_ws/src/objectdetection_msgs/Bin.pt', force_reload=True)

# Load the COCO-pretrained YOLOv5 model
model_v5s = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# model_v5s = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/dhaval_lad/dhaval_ws/src/object_detection/object_detection/yolov5s.pt', force_reload=True)

# Define confidence thresholds for different objects
CONF_THRESHOLDS_COUSTOM = {
    'Dustbin': 0.85
}

# Define confidence thresholds for COCO classes
CONF_THRESHOLDS = {
    'person': 0.5,
    'bicycle': 0.6
}

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(ObjectDetectionList, 'detections_publisher', 1)
        self.bridge = CvBridge()

        # Subscriptions to Gazebo camera topics
        self.subscription_rgb = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1)
        
        self.lidar_subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1)
        self.laser_angle_increment = 1.0 * np.pi / 180  # radians
        self.latest_scan = np.array([np.float32(10)] * 270)    
        self.laser_reads_front = np.array([np.float32(10)] * 80) 

        self.color_image = None

        frequency = 10  
        self.timer = self.create_timer(1/frequency, self.timer_callback)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def scan_callback(self, scan_msg: LaserScan):
        self.laser_angle_increment = scan_msg.angle_increment
        self.latest_scan = np.array(scan_msg.ranges)
        self.latest_scan[self.latest_scan == np.inf] = np.float32(10)
        self.laser_reads_front = np.array(self.latest_scan[50:130]) 

    def timer_callback(self):
        if self.color_image is None:
            return  # Skip processing if images are not available yet

        # Define the width and height of the ROI
        roi_width = 500  # Adjust this value to extend the horizontal span
        roi_height = 300  # Adjust this value to change the vertical span if needed

        # Calculate the center of the image
        center_x = self.color_image.shape[1] // 2
        center_y = self.color_image.shape[0] // 2

        # Calculate the coordinates of the ROI
        x1_roi = max(center_x - roi_width // 2, 0)
        y1_roi = max(center_y - roi_height // 2, 0)
        x2_roi = min(center_x + roi_width // 2, self.color_image.shape[1])
        y2_roi = min(center_y + roi_height // 2, self.color_image.shape[0])

        # Draw the ROI box on the original image for visualization
        cv2.rectangle(self.color_image, (x1_roi, y1_roi), (x2_roi, y2_roi), (0, 255, 0), 2)

        # Crop the color images accprding to the ROI
        cropped_color_image = self.color_image[y1_roi:y2_roi, x1_roi:x2_roi]

        # Detect objects using YOLOv5 on the cropped image
        results = model_v5s(cropped_color_image)

        detected_objects = []  # List to hold detected object names

        # Process the results for COCO-pretrained model
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = result

            class_name = model_v5s.names[int(class_id)]
            confidence_threshold = CONF_THRESHOLDS.get(class_name, 0.55)

            if class_name == "person" or class_name == 'bicycle':
                if confidence > confidence_threshold:
                    # Adjust coordinates to the full image
                    x1_full = x1 + x1_roi
                    y1_full = y1 + y1_roi
                    x2_full = x2 + x1_roi
                    y2_full = y2 + y1_roi

                    # Calculate the distance to the object
                    x_centers = (x1_full + x2_full) / 2         # center of bounding box in horizontal
                    y_center = (y1_full + y2_full) / 2          # center of bounding box in vertical
                    x_center = 320 - x_centers                  # center with respect to horizontal FoV of camera
                    pixel_size_meters = 0.00155
                    x_center = x_center * pixel_size_meters
                    
                    # Calculate the distance to the object
                    # camera cover 80° FoV over Lidar range. which mean 80 laser read in 640 pixels. 
                    center = -int(x_centers*0.125)              # 80/640=0.125 (laser read/horizontal pixel) 
                    # print("Person Center: ", center)
                    index_for_min = np.argmin(self.laser_reads_front[center-5:center+5])
                    # print("index person: ",index_for_min)

                    # calculate angle of the object detected from right hand side of the robot. 
                    object_angle = ((80.0+center-5.0+index_for_min)+50.0)    # here 50° is added because Lidar ranges starts from right side and camera FOV range from 50° to 130°. 
                                                                                            # and 80° is foe complete FoV of camera.   
                    # print("Person Angle: ",object_angle)

                    # extract depth value from the Lidar ranges with the help of 'center'
                    object_depth = float(min(self.laser_reads_front[center-5:center+5]))
                    # print("Person depth: ", object_depth)

                    label = f"({model_v5s.names[int(class_id)]},{object_depth:.2f}m,{object_angle:.2f})"

                    # Create ObjectDetection message
                    detection_msg = ObjectDetection()
                    detection_msg.object_class = class_name
                    detection_msg.distance = object_depth
                    detection_msg.angle = object_angle

                    # Add object class to the list
                    detected_objects.append(detection_msg)

                    # Draw a rectangle around the object on the full image
                    cv2.rectangle(self.color_image, (int(x1_full), int(y1_full)), (int(x2_full), int(y2_full)), (252, 119, 30), 2)
                    cv2.circle(self.color_image, (int(x_centers), int(y_center)), 5, (0, 0, 255), -1)

                    # Draw the bounding box with label
                    cv2.putText(self.color_image, label, (int(x1_full), int(y1_full)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

                    # Print the object's class and distance
                    print(f"{model_v5s.names[int(class_id)]}: {label}")

        # Detect objects using custom model on the cropped image
        results_custom = model_custom(cropped_color_image)

        # Process the results for the custom model
        for result in results_custom.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = result

            class_name = model_custom.names[int(class_id)]
            confidence_threshold = CONF_THRESHOLDS_COUSTOM.get(class_name, 0.55)
            # print(f"Detected '{class_name}' with confidence {confidence:.2f}. Threshold for '{class_name}': {confidence_threshold:.2f}")

            if class_name == "Dustbin":
                if confidence > confidence_threshold:
                    # Adjust coordinates to the full image
                    x1_full = x1 + x1_roi
                    y1_full = y1 + y1_roi
                    x2_full = x2 + x1_roi
                    y2_full = y2 + y1_roi

                    # Calculate the distance to the object
                    x_centers = (x1_full + x2_full) / 2         # center of bounding box in horizontal
                    y_center = (y1_full + y2_full) / 2          # center of bounding box in vertical
                    x_center = 320 - x_centers                  # center with respect to horizontal FoV of camera
                    pixel_size_meters = 0.00155
                    x_center = x_center * pixel_size_meters
                   
                    # Calculate the distance to the object
                    # camera cover 80° FoV over Lidar range. which mean 80 laser read in 640 pixels. 
                    center = -int(x_centers*0.125)              # 80/640=0.125 (laser read/horizontal pixel) 
                    # print("Dustbin Center: ", center)

                    # calculate angle of the object detected from right hand side of the robot. 
                    object_angle = ((80.0+center) + 50.0)   # here 50° is added because Lidar ranges starts from right side and camera FOV range from 50° to 130°. 
                                                                                            # and 80° is foe complete FoV of camera.   
                    # print("Dustbin Angle: ", object_angle)

                    # extract depth value from the Lidar ranges with the help of 'center'
                    object_depth = float(self.laser_reads_front[center])  
                    # print("Dustbin depth: ", object_depth)

                    label = f"({model_custom.names[int(class_id)]},{object_depth:.2f}m,{object_angle:.2f})"

                    # Create ObjectDetection message
                    detection_msg = ObjectDetection()
                    detection_msg.object_class = class_name
                    detection_msg.distance = object_depth
                    detection_msg.angle = object_angle

                    # Add object class to the list
                    detected_objects.append(detection_msg)

                    # Draw a rectangle around the object on the full image
                    cv2.rectangle(self.color_image, (int(x1_full), int(y1_full)), (int(x2_full), int(y2_full)), (252, 119, 30), 2)
                    cv2.circle(self.color_image, (int(x_centers), int(y_center)), 5, (0, 0, 255), -1)

                    # Draw the bounding box with label
                    cv2.putText(self.color_image, label, (int(x1_full), int(y1_full)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

                    # Print the object's class and distance
                    print(f"{model_custom.names[int(class_id)]}: {label}")

        # Publish detected objects to the topic
        detection_list_msg = ObjectDetectionList()
        detection_list_msg.detections = detected_objects
        self.publisher_.publish(detection_list_msg)

        # Show the image with the ROI box drawn
        cv2.imshow("Color Image", self.color_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly (optional)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
