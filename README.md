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

## **시스템 구성**
### yolov5<br/>
  * 신호등에서 빨간불이면 정차되고, 파란불이면 주행이 지속됩니다.<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![traffic](https://github.com/user-attachments/assets/3325e0f3-8c5d-4c82-a0a7-7be784703e73)<br/>
  * 검출된 직진 화살표의 중심점을 따라 조향각을 측정합니다.<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![up](https://github.com/user-attachments/assets/31a45dc7-18eb-40d6-99ba-eb056b66a686)
  <br/><br/>

  ### lane_detection<br/>
  * 영상을 HSV값으로 입력받아 차선 색상값을 기준으로 이진화를 수행합니다. 이후 CannyEdge를 수행하여 허프변환을 통해 차선을 검출하고 조향각을 측정합니다.<br/>
  * 급커브 구간에는 차선이 2개로 검출이 되지않아 안정적인 주행이 어렵습니다. 이를 보완하기 위해 조향선(빨간 직선)과 차선에서 1개로 검출된 직선(녹색 직선)이 교차 될 경우 해당 사잇각을 측정하여 조향각을 산출합니다. 이후 2개의 차선이 검출될때 까지 산출된 조향값이 지속됩니다.<br/><br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![corner_car](https://github.com/user-attachments/assets/e47474e8-4d9f-457a-be03-36fc9e5626a0)
  ![corner_car_2](https://github.com/user-attachments/assets/db806a82-9a8e-496d-b097-b1e1c86c8aa5)
  ![corner](https://github.com/user-attachments/assets/b60fbfa3-19ae-44d1-b356-1e43c62b72bb)
  <br/><br/>
  
  ### rplidar_ros<br/>
  * 장애물 감지를 위해 라이다 센서를 활용하였으며 일정거리 미만일 경우 정차되고, 이상일 경우 주행이 지속됩니다.<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![lidar_stop_car](https://github.com/user-attachments/assets/8afdf4c6-7ab2-4ce3-9058-aea1ad58eadf)
  ![lidar_stop](https://github.com/user-attachments/assets/8510b564-6831-4b4b-8025-5e3552905c56)
  <br/><br/>

  ### crosswalk<br/>
  * 영상을 이진화와 CannyEdge 수행하여 정지선이 검출되면 일정시간 동안 정차되고, 이후 횡단보도에 사람이 없으면 주행이 지속됩니다.<br/>
  &nbsp;&nbsp;&nbsp;&nbsp;![crosswalk_car](https://github.com/user-attachments/assets/934a6ca3-5cdd-45b0-a8b3-636f9b35682c)
  ![crosswalk](https://github.com/user-attachments/assets/351b5768-9822-40c2-b23f-ddae6693ef13)
  <br/><br/>

  ### control_manager<br/>
  * yolov5,lane_detection,rplidar_ros,crosswalk의 결과값을 입력받아 속도와 조향각을 결정합니다.<br/><br/>  
  ### car_control(Arduino)<br/>
  * control_manager의 제어값을 통해 차량제어가 수행됩니다.
