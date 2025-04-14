# Franka Handeye Calibration

This repo is tested with **Ros2 Humble Ubuntu 22.04** & **FR3** & **Realsense 455**, which can reallize eye_on_base or eye_on_hand calibration with Aruco QR code. One has to manually control the franka moveit to move the robot,  then sample 6 or more data to acquire relatively precise transpose result. About ~1.5cm deviation, which can be manually correct during using.

Related Repos: [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2), [ros2_aruco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco), [libfranka](https://github.com/frankaemika/libfranka), [franka_ros2](https://github.com/frankaemika/franka_ros2), [librealsense](https://github.com/IntelRealSense/librealsense), [realsense_ros](https://github.com/IntelRealSense/realsense-ros).

Renference Repo: [ar4_hand_eye_calibration](https://github.com/ycheng517/ar4_hand_eye_calibration). Appreciate for this author's work.

>Ros1 Neotic Franka Handeye Calbration: [Handeye Calibration](https://mixed-raccoon-b7c.notion.site/Handeye-Calibration-17eff737deb5800181d3f0844ec56095?pvs=4)
Hardware: Franka1, realsense d435, aruco

# Preparation
1. Find an aruco marker on this [websit](https://chev.me/arucogen/), and attach it to the franka end-effector. If you don't want to change the code, this repo use aruco marker information:
    ```
    Dictionary: 5x5(50, 100, 250, 1000);
    Marker ID: 250;
    Marker size, mm: 50
    ```
    If you change the information, remember to change the corresponding part in `config/aruco_parameters.yaml`

2. Install [Ros2 humble](https://docs.ros.org/en/humble/Installation.html)
3. Install [libfranka](https://github.com/frankaemika/libfranka) and [franka_ros2](https://github.com/frankaemika/franka_ros2). Remember to choose the corresponding version from [here](https://frankaemika.github.io/docs/compatibility.html) of your robot firmware.
4. Install [librealsense](https://github.com/IntelRealSense/librealsense) & [realsense_ros](https://github.com/IntelRealSense/realsense-ros) from source (Remember to choose the corresponding version. This repo uses librealsense: v2.55.1,realsense_ros: 4.55.1). Or use command:
    ```
    sudo apt install ros-iron-librealsense2* ros-iron-realsense2-*
    ```

# Installation
Check the Aruco and Easy_Handeye git version, incase they update their repo to Ubuntu24
```
mkdir -p ~/calib_ws/src
cd ~/calib_ws/src

git clone https://github.com/ChengYaofeng/franka_handeye_calibration_ros2.git

git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git

git clone https://github.com/marcoesposito1988/easy_handeye2

git clone https://github.com/AndrejOrsula/pymoveit2.git

rosdep install -iyr --from-paths src

colcon build
```

# Error & Solutions
1. `ModuleNotFoundError: No module named 'tf_transformations'`
    ```
    sudo apt-get install ros-humble-tf-transformations
    ```
2. `[aruco_node-1] AttributeError: module 'cv2.aruco' has no attribute 'Dictionary_get'. Did you mean: 'Dictionary'?`

    The opencv-version is to high. `pip uninstall opencv-python`. Directly use the system version.

# Execution
Activate FR3 Moveit, adjust the command to your own robot ip.
```
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:='172.16.0.2'
```
In a new terminal, launch the calibration file. You need to put the camera to capture the aruco marker on the robot before running the following command. Otherwise you'll meet the error.
```
ros2 launch franka_handeye_calib calibrate.launch.py 
```
Then use the rviz to move the robot 6 or more times to acquire relatively precise transpose result. About ~1.5cm deviation, which can be manually correct during using.

The final results will saved in ~/.ros2/easy_handeye/calibrations/fr3_calibration.calib
