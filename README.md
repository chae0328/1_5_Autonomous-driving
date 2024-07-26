# 1_5_Autonomous-driving
```  
${ROOT}  
|-- main_ws
|   |-- build
|   |-- devel
|   |-- src
|   |   |-- car_control
|   |   |   |-- car_control.ino
|   |   |-- rplidar_ros
|   |   |-- vision_control
|   |   |   |-- include
|   |   |   |-- launch
|   |   |   |   |-- control_manager.launch
|   |   |   |-- scripts
|   |   |   |   |-- yolov5
|   |   |   |   |-- control_manager.py
|   |   |   |   |-- crosswalk.py
|   |   |   |   |-- lane_detect.py
```  

### **시스템구성**
* lane_detect<br/><br/>
  영상을 HSV값으로 입력받아 차선 색깔에 맞춰 이진화를 수행합니다 이후 CannyEdge 검출을 하여 허프변환을 통해 차선을 검출하여 조향각을 계산합니다.
![corner](https://github.com/user-attachments/assets/b60fbfa3-19ae-44d1-b356-1e43c62b72bb)
![corner_car](https://github.com/user-attachments/assets/e47474e8-4d9f-457a-be03-36fc9e5626a0)
![corner_car_2](https://github.com/user-attachments/assets/db806a82-9a8e-496d-b097-b1e1c86c8aa5)
![crosswalk](https://github.com/user-attachments/assets/53ac2b06-0974-44d7-a949-90a870c90cf4)
![crosswalk_car](https://github.com/user-attachments/assets/253a164b-0b6a-49f9-b079-abf5d82b2063)
* crosswalk
    
* control_manager

* car_control<br/><br/>속도와 회전해야할 조향 각도를 아두이노에 입력받아 차량제어가 진행됩니다.
