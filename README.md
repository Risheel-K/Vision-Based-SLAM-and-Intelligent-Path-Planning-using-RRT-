# Vision-Based-SLAM-and-Intelligent-Path-Planning-using-RRT-
A compact autonomous robot using Raspberry Pi integrates SLAM, YOLOv5 for real-time human detection, and RRT* for optimal path planning. It maps surroundings, detects a person as a goal, and navigates safely. Built in Python, it's lightweight and ideal for rescue, assistance, and indoor navigation.

This project focuses on the development of a vision-based autonomous robotic system capable of real-time human detection and intelligent path planning in unstructured environments. The system is designed to operate fully on a compact embedded platform — the Raspberry Pi — making it portable, cost-effective, and free from reliance on external microcontrollers or middleware.

At its core, the robot integrates three main modules:

    Visual Perception and Human Detection:
    Using a USB camera connected to the Raspberry Pi, the robot captures live video frames. These frames are processed using YOLOv5, a state-of-the-art real-time object detection model. The model accurately detects humans within the frame and extracts their position in the environment, which is then dynamically set as the navigation goal for the robot.

    Simultaneous Localization and Mapping (SLAM):
    As the robot moves, it simultaneously constructs a basic map of the surroundings using visual input. This helps the robot localize its current position while identifying static obstacles. This visual SLAM approach allows the robot to understand and adapt to unknown, dynamic environments without preloaded maps.

    RRT* Path Planning and Obstacle Avoidance:
    Once the human target is identified, the system uses the RRT* (Rapidly-exploring Random Tree Star) algorithm to compute the shortest collision-free path to reach the target. RRT* ensures optimality by continuously refining the path as new obstacles are detected, offering smooth and efficient navigation even in randomly cluttered environments.

Software Architecture:
The entire software stack is modular and written in Python. It includes:

    Image processing pipeline using OpenCV and PyTorch

    Object detection using pretrained YOLOv5 models

    SLAM integration for map generation

    Real-time RRT* path planner

    Visualizer to monitor robot movement and map status

Execution Environment:

    Hardware: Raspberry Pi 3/4, USB camera

    Software: Python 3.x, OpenCV, PyTorch, NumPy, Matplotlib

    No external microcontrollers or ROS middleware are used

Key Features:

    Fully onboard processing using Raspberry Pi

    Real-time person detection and goal-locking

    On-the-fly mapping and obstacle recognition

    Dynamic and optimized path planning

    Live visualization for monitoring and debugging

Applications:

    Search and rescue operations in indoor/unknown spaces

    Human-following assistant robots

    Smart service robots in hospitals, malls, and homes

    Educational and research platform for vision-based robotics

This project showcases the synergy of computer vision, embedded AI, and intelligent path planning to build a low-cost autonomous navigation system that functions reliably in real-world environments.

There is a camindex file used to automatically detect and set the correct camera index for the system. This ensures the code dynamically selects the connected camera without manual changes. Additionally, using a USB camera provides higher resolution and more accurate visual data, enabling precise detection and calculations.

In this project, we used a Raspberry Pi 4 as the main processing unit, along with 12V DC motors controlled by BTS7960 motor drivers. A USB camera was integrated for real-time vision, enabling high-resolution image capture for accurate detection and navigation.

Circuit diagram of the Raspberry pi ![SCH](https://github.com/user-attachments/assets/d28b0f49-f8f1-4612-9ca2-f651adf80c99)

