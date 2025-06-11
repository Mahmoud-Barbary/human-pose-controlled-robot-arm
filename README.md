# Human-Pose Controlled Robotic Arm
A real-time, vision-based system for controlling a 6-DOF robotic arm using human pose estimation. Built with MediaPipe and ROS MoveIt, this project explores intuitive human-robot collaboration without wearable sensors, demonstrating a simple but extensible framework for robotic motion replication using shoulder and elbow joint tracking.

## 🖼️ Demo

## 🚀 Overview

- **Vision-Based Control**: Tracks human arm movement via webcam.
- **Robot Replication**: Sends joint angles to a Kinova Gen3 arm.
- **Real-Time Execution**: Maps human elbow/shoulder angles to robot joints.
- **Simulation + Real Hardware**: Initially tested in Gazebo, then ported to physical robot.

## 📷 How It Works

1. **MediaPipe** identifies human landmarks (shoulders, elbows, wrists).
2. The script computes **elbow and shoulder angles** from 2D vectors.
3. Angles are mapped to the robot's joints using **MoveIt!**
4. The robot arm moves to match the estimated angles with a safe tolerance.

## 📁 File Structure

- `pose_to_kinova.py`: Main Python script (ROS node) that controls the robot based on human pose.
- `/images/`: Screenshots of simulation and real-world demos *(optional)*.
- `README.md`: This file.
- (Optional) `requirements.txt`: List of Python packages (e.g. `mediapipe`, `opencv-python`, `numpy`).

## ⚠️ Limitations
- 2D pose tracking lacks depth info, so motion estimation isn't perfect.
- Requires well-lit, front-facing camera setup for reliable detection.
- Limited to shoulder and elbow angles for simplicity.

## 📌 Future Directions
- Add depth sensing (3D pose estimation)
- Modularize for different robotic arms
- Add object manipulation via addition of robotic claw
- Improve latency and motion prediction

## 🛠 Dependencies
- **Python 3**
- **ROS Noetic / Melodic** with MoveIt!
- **Kinova Gen3** with appropriate drivers
- `cv2`, `mediapipe`, `numpy`

## 📚 Project Report
For a full write-up of the design, methodology, results, and lessons learned, see this PDF.

Project for CSCI 5551 - Introduction to Robotics, University of Minnesota, Spring 2025.
