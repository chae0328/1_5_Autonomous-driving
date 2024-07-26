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
  영상을 HSV값으로 입력받아 차선 색깔에 맞춰 이진화를 수행합니다 이후 CannyEdge 검출을 하여 허프변환을통해 차선을 검출하고 조향각을 조절합니다. 
  
* control_manager
* crosswalk
* car_control<br/><br/>속도와 회전해야할 조향 각도를 아두이노에 입력받아 차량제어가 진행됩니다.
