# 3_wheel_robot

1. flash sd card
2. แก้ไฟล์ใน sd card เพื่อใส่รหัส wifi
3. เปิด rpi เพื่อดู ip address ด้วย ifconfig
4. ดูเลขที่ wlan0 - inet รูป
5. ssh ubnutu@10.61.2.19
6. yes ถ้าครั้งแรก แล้วใส่รหัส

``` https://github.com/xinjuezou-whi/rplidar_ros2 ```
```https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/```

สร้าง package
1.ros2 pkg create --build-type ament_cmake robot_description

2. nano CMakeLists.txt 

3. install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
