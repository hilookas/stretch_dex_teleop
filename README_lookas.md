```bash
python -m venv venv
# Upgrade OpenCV 
# Ref: https://stackoverflow.com/questions/75085270/cv2-aruco-charucoboard-create-not-found-in-opencv-4-7-0
pip install opencv-python pyyaml pyusb numba urchin scipy
apt install ros-humble-xacro python3.10-venv ffmpeg v4l-utils
pyrender





# 加载urdf模型
ros2 run robot_state_publisher robot_state_publisher ~/ros2_ws/non_ros_src/stretch_urdf/stretch_urdf/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3___abs_path.urdf

# 调参界面
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```