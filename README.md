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
|   |   |   |   |-- lane_detection.py
```  

## **시스템구성**
### yolov5<br/>
  * 신호등에서 빨간불 신호면 정차, 파란불 들어오면 주행을 수행합니다.<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![traffic](https://github.com/user-attachments/assets/3325e0f3-8c5d-4c82-a0a7-7be784703e73)<br/>
  * 직진 화살표가 검출이되면 객체의 중심점을 따라 조향각을 계산합니다.<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![up](https://github.com/user-attachments/assets/31a45dc7-18eb-40d6-99ba-eb056b66a686)
  <br/><br/>

  ### lane_detection<br/>
  * 영상을 HSV값으로 입력받아 차선 색상값을 기준으로 이진화를 수행합니다 이후 CannyEdge 검출을 하여 허프변환을 통해 차선을 검출하고 조향각을 계산합니다.<br/><br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![corner_car](https://github.com/user-attachments/assets/e47474e8-4d9f-457a-be03-36fc9e5626a0)
  ![corner_car_2](https://github.com/user-attachments/assets/db806a82-9a8e-496d-b097-b1e1c86c8aa5)
  ![corner](https://github.com/user-attachments/assets/b60fbfa3-19ae-44d1-b356-1e43c62b72bb)
  <br/><br/>
  
  ### rplidar_ros<br/>
  * (작성)<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![lidar_stop_car](https://github.com/user-attachments/assets/8afdf4c6-7ab2-4ce3-9058-aea1ad58eadf)
  ![lidar_stop](https://github.com/user-attachments/assets/8510b564-6831-4b4b-8025-5e3552905c56)
  <br/><br/>

  ### crosswalk<br/>
  * (작성)<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![crosswalk_car](https://github.com/user-attachments/assets/934a6ca3-5cdd-45b0-a8b3-636f9b35682c)
  ![crosswalk](https://github.com/user-attachments/assets/351b5768-9822-40c2-b23f-ddae6693ef13)
  <br/><br/>

  ### control_manager<br/>
  * pass<br/>
  <br/><br/>
  
  ### car_control<br/>
  * 속도와 회전해야할 조향 각도를 아두이노에 입력받아 차량제어가 진행됩니다.
  <br/><br/>
