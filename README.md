# 3_wheel_robot
## PC Setup
1. install ubuntu 22.04
2. install ros2 humble
3. echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
4. source ~/.bashrc

## Single-Board Computer (SBC) Setup
1. ทำตามขั้นตอนใน https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup [อย่าลืมเลือกเป็น humble] [ไม่จำเป็นต้องทำขั้นตอนเกี่ยวกับ turtlebot3 หรือ dynamixel
2. เมื่อ setup เรียบร้อย เปิด rpi เพื่อดู ip address ด้วย ifconfig
3. ดูเลข ip ที่ wlp - inet [ip]
4. ssh ubnutu@10.61.2.19 ที่ PC เพื่อให้ PC สามารถ remote เข้าไปทำได้ [ถ้าค้างต้องแก้ตามนี้ https://johnpili.com/fix-raspberry-pi-ssh-freezing-issue/]
5. ใส่รหัสที่เคยตั้งไว้ 
6. clone package ลงใน your_ws/src
```
  git clone https://github.com/SuaPiamsuk/3_wheel_robot.git
```

## Guide how to use ROS with Arduino
https://github.com/ResQBots/resqbot_drive_interface

## Source
### 1. Turtlebot3
Website: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Github: https://github.com/ROBOTIS-GIT/turtlebot3

### 2. RPLIDAR for ROS2 Humble
https://github.com/babakhani/rplidar_ros2
### 3. RPLIDAR for ROS2 Foxy
https://github.com/Kp2499/rplidar_ros2
### 4. Create a ROS2 package for Both Python and Cpp Nodes
https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/
### 5. IMU sensor [MPU9250]
https://microcontrollerslab.com/esp32-mpu9250-tutorial/
### 6. Micro-ros
https://www.youtube.com/watch?v=fo5I9ZYbG5Q

https://cps.unileoben.ac.at/category/wiki_hard_software/?print=print-search
