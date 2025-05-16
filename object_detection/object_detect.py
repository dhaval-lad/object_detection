#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from objectdetection_msgs.msg import ObjectDetection, ObjectDetectionList

# Load the YOLOv5 custom model with object class "Dustbin"
model_custom = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/dhaval_lad/dhaval_ws/src/object_detection/object_detection/Bin.pt', force_reload=True)

# Load the COCO-pretrained YOLOv5 model
model_v5s = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# model_v5s = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/dhaval_lad/dhaval_ws/src/object_detection/object_detection/yolov5s.pt', force_reload=True)


# Set up the RealSense D455 camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Write your yolov5 depth scale here
depth_scale = 0.001

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
        self.publisher_ = self.create_publisher(ObjectDetectionList, 'detections_publisher', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Get the latest frame from the camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convert the frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert the depth image to meters
        depth_image = depth_image * depth_scale

        # Detect objects using YOLOv5
        results = model_v5s(color_image)

        # List to hold detected object names
        detected_objects = []  

        # Process the results
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = result

            class_name = model_v5s.names[int(class_id)]
            # print(f"Detected class name: {class_name}")  # Debugging print statement

            confidence_threshold = CONF_THRESHOLDS.get(class_name, 0.55)  # Default threshold if class is not in dictionary
            # Print the confidence threshold for the detected object
            # print(f"Detected '{class_name}' with confidence {confidence:.2f}. Threshold for '{class_name}': {confidence_threshold:.2f}")

            if class_name == "person" or class_name =='bicycle':
                if confidence > confidence_threshold:
                    # Calculate the distance to the object directly from the depth image. 
                    object_depth = np.median(depth_image[int(y1):int(y2), int(x1):int(x2)])

                    x_centers = (x1 + x2) / 2                    # center of bounding box in horizontal
                    y_center = (y1 + y2) / 2                     # center of bounding box in vertical
                    x_center = 320 - x_centers                   # center with respect to horizontal FoV of camera
                    pixel_size_meters = 0.00155
                    x_center = x_center * pixel_size_meters

                    # calculate angle of the object detected from right hand side of the robot. 
                    fov_half = 43.5  # Half of horizontal FOV of RealSense D455
                    angle = 90 - ((x_centers - 320) / 320) * fov_half  
                    
                    label = f"({model_v5s.names[int(class_id)]},{object_depth:.2f}m,{angle:.2f})"

                    # Create ObjectDetection message
                    detection_msg = ObjectDetection()
                    detection_msg.object_class = class_name
                    detection_msg.distance = object_depth
                    detection_msg.angle = float(angle)

                    # Add object class to the list
                    detected_objects.append(detection_msg)

                    # Draw a rectangle around the object
                    cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (252, 119, 30), 2)
                    cv2.circle(color_image, (int(x_centers), int(y_center)), 5, (0, 0, 255), -1)

                    # Draw the bounding box with label
                    cv2.putText(color_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

                    # Print the object's class and distance
                    print(f"{model_v5s.names[int(class_id)]}: {label}")

        # Detect objects using coustom model
        results_coustom = model_custom(color_image)
        # print(len(results.xyxy[0]))

        # Process the results
        for results_coustom in results_coustom.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = results_coustom

            class_name = model_custom.names[int(class_id)]
            # print(f"Detected class name: {class_name}")  # Debugging print statement

            confidence_threshold = CONF_THRESHOLDS_COUSTOM.get(class_name, 0.55)  # Default threshold if class is not in dictionary
            # Print the confidence threshold for the detected object
            # print(f"Detected '{class_name}' with confidence {confidence:.2f}. Threshold for '{class_name}': {confidence_threshold:.2f}")

            if class_name == "Dustbin":
                if confidence > confidence_threshold:
                    # Calculate the distance to the object directly from the depth image.
                    object_depth = np.median(depth_image[int(y1):int(y2), int(x1):int(x2)])

                    x_centers = (x1 + x2) / 2                    # center of bounding box in horizontal
                    y_center = (y1 + y2) / 2                     # center of bounding box in vertical
                    x_center = 320 - x_centers                   # center with respect to horizontal FoV of camera
                    pixel_size_meters = 0.00155
                    x_center = x_center * pixel_size_meters

                    # calculate angle of the object detected from right hand side of the robot. 
                    fov_half = 43.5  # Half of horizontal FOV of RealSense D435i
                    angle = 90 - ((x_centers - 320) / 320) * fov_half  # >>> Angle Calculation
                    
                    label = f"({model_custom.names[int(class_id)]},{object_depth:.2f}m,{angle:.2f})"

                    # Create ObjectDetection message
                    detection_msg = ObjectDetection()
                    detection_msg.object_class = class_name
                    detection_msg.distance = object_depth 
                    detection_msg.angle = float(angle)

                    # Add to the list of detected objects
                    detected_objects.append(detection_msg)

                    # Draw a rectangle around the object
                    cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (252, 119, 30), 2)
                    cv2.circle(color_image, (int(x_centers), int(y_center)), 5, (0, 0, 255), -1)

                    # Draw the bounding box with label
                    cv2.putText(color_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

                    # Print the object's class and distance
                    print(f"{model_custom.names[int(class_id)]}: {label}")

        # Publish detected objects to the topic
        detection_list_msg = ObjectDetectionList()
        detection_list_msg.detections = detected_objects
        self.publisher_.publish(detection_list_msg)

        # Show the image
        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
