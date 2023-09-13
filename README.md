# 3_wheel_robot

1. flash sd card
2. แก้ไฟล์ใน sd card เพื่อใส่รหัส wifi
3. เปิด rpi เพื่อดู ip address ด้วย ifconfig
4. ดูเลขที่ wlan0 - inet รูป
5. ssh ubnutu@10.61.2.19 ถ้าค้างต้องแก้ตามนี้ https://johnpili.com/fix-raspberry-pi-ssh-freezing-issue/
6. yes ถ้าครั้งแรก แล้วใส่รหัส

``` https://github.com/Kp2499/rplidar_ros2 ```
```https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/```

สร้าง package
1.ros2 pkg create --build-type ament_cmake robot_description

2. nano CMakeLists.txt 

3. install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

# mpu9250

https://microcontrollerslab.com/esp32-mpu9250-tutorial/

# micro-ros
https://www.youtube.com/watch?v=fo5I9ZYbG5Q
https://cps.unileoben.ac.at/category/wiki_hard_software/?print=print-search
