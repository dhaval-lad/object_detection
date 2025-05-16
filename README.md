# object_detection

## Description
The `object_detection` package is responsible for processing camera and LiDAR data to detect object 'class', 'depth' and 'respective angle'. This package uses Camera-LiDAR fusion for to get respective information in simulation. And in real Intel RealSense D435i these informations are directly calculated from camera messages. 

## Dependencies
- ROS 2
- [objectdetection_msgs](https://github.com/dhaval-lad/objectdetection_msgs.git) : Custom msgs for object detection, depth and angle.
- Intel RealSense D435i.
- A robot with Depth camera and 2D LiDAR (270° FoV). Recomended: [GrassHopper Gazebo](https://github.com/dhaval-lad/grasshopper_gazebo.git)

## Installation Instructions
1. First, clone this repository inside the `src` folder of your ROS 2 workspace (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws/src
    git clone https://github.com/dhaval-lad/object_detection.git
    ```
2. Download the required `Bin.pt` file and move it to the appropriate location:
    - Download the file manually from Google Drive: [Download Bin.pt](https://drive.google.com/file/d/1NCyYoV9lyguDxOITWGhy7poy5eQ7BJMZ/view?usp=drive_link)
    - After downloading (typically saved in your `~/Downloads` folder), move it to the correct directory using the command below. (Be sure to replace `your_username` with your actual Ubuntu username and `your_ros2_ws` with your actual ROS 2 workspace name):
    ```
    mv ~/Downloads/Bin.pt /home/your_username/your_ros2_ws/src/objectdetection_msgs/Bin.pt
    ```
3. Update the path to the Bin.pt file in the following scripts with your actual file path:
    - In object_detect.py (line 11)
    - In object_detect_gazebo.py (line 12)
    
    Replace the existing path with:
    ```
    /home/your_username/your_ros2_ws/src/objectdetection_msgs/Bin.pt
    ```
4. Next, build your ROS 2 workspace to install the package (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select object_detection
    ```

## Usage Instructions
This can be varified either in simulation or with IntelRealsense D435i.
### Simulation Setup
To verify that everything is working in simulation:
1. Run the Gazebo world with the robot in it. You can use GrassHopper simulation model available at: [https://github.com/dhaval-lad/grasshopper_gazebo.git](https://github.com/dhaval-lad/grasshopper_gazebo.git)
2. Change the topic name of `/camera/image_raw` and `/scan` in `object_detecet_gazebo.py` to match the topic name in your simulation environment.
3. Rebuild the package:
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select object_detection
    ```
4. Run the `object_detect_gazebo` node to see the object classification is working:
    ```sh
    ros2 run object_detection object_detect_gazebo
    ```

### Real World Setup
To verify that everything is working onGrassHopper:
1. Connect the Intel RealSense D435i to the system where this pkg is.
2. Run the `object_detect` node to see the object classification is working:
    ```sh
    ros2 run object_detection object_detect
    ```

## Nodes
- **object_detect**: Responsible for reading the data from Intel RealSense D435i through `pyrealsense2` pipeline and process it to get object class, depth and angle with respect to robot. 
- **object_detect_gazebo**: Responsible for subscribing to `/camera/image_raw` and `/scan` from simulation environment, and publishing on `/detections_publisher` the class, depth and the angle where the object is detected.

## License
This project is licensed under the Apache License, Version 2.0, January 2004. See [http://www.apache.org/licenses/](http://www.apache.org/licenses/) for more details.
