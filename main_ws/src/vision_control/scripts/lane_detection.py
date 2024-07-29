#!/usr/bin/env python3
import cv2
import numpy as np
import logging
import math
import datetime
import sys
import threading
import time
import torch
sys.path.append('yolov5')
from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes, check_img_size
from utils.augmentations import letterbox

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

#publish_변수
angle_pub = None
object_pub = None
#차선이 검출될 기울기 임계값 설정
min_slope = 0.4

##############################
# 콘트라스트 및 밝기 조절  (1.0 ~ 3.0), (0, 100) 
alpha,beta = 1.0, 0
##############################

Turn_Flag = False
prev_turn_angle = 0
max_angle = 20
weights = 'traffic.pt'
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
label_coordinate = [] #yolo 결과삽입

def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue1 = np.array([70, 100, 100])
    upper_blue1 = np.array([110, 255, 255])
    mask1 = cv2.inRange(hsv, lower_blue1, upper_blue1)
    lower_blue2 = np.array([110, 100, 100])
    upper_blue2 = np.array([140, 255, 255])
    mask2 = cv2.inRange(hsv, lower_blue2, upper_blue2)
    mask = mask1 + mask2
    edges = cv2.Canny(mask, 200, 400)
    return edges

def region_of_interest(canny):
    height, width = canny.shape
    mask = np.zeros_like(canny)
    polygon = np.array([[
        (0, height * (1 / 2)),
        (width, height * (1 / 2)),
        (width, height),
        (0, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

def detect_line_segments(cropped_edges):
    rho = 1
    angle = np.pi / 180
    min_threshold = 10
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=15, maxLineGap=4)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    #기울기
                    if slope < -min_slope:
                        left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    #기울기 기존값 0.75
                    if slope > min_slope:
                        right_fit.append((slope, intercept))

    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(make_points(frame, left_fit_average))

    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def compute_steering_angle(frame, lane_lines):
    if len(lane_lines) == 0:
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    if num_of_lane_lines == 2:
        max_angle_deviation = max_angle_deviation_two_lines
    else:
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height
    y2 = int(y1 * 1 / 2)

    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def draw_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(frame, (x1, y1), (x2, y2), line_color, line_width)

def ie(x1, y1, x2, y2, x3, y3): #교차점 판단 intersection_estimation
    """
    세 점이 반시계 방향인지, 시계 방향인지, 혹은 일직선 상에 있는지 판단
    반환값:
        1: 반시계 방향
        -1: 시계 방향
        0: 일직선 상에 있음
    """
    ans = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
    if ans < 0:
        return 1
    elif ans > 0:
        return -1
    else:
        return 0
    
def ip(x1,y1,x2,y2,x3,y3,x4,y4): #교차점 계산 intersection_point (steering_point, lane_point)
    
    #기울기 절편 구하기
    slope1 = (y2 - y1) / (x2 - x1)
    intercept1 = y1 - slope1 * x1
    slope2 = (y4 - y3) / (x4 - x3) #lane_point_slope
    intercept2 = y3 - slope2 * x3
    #교점의 x,y 좌표 계산
    x = (intercept2 - intercept1) / (slope1 - slope2) 
    y = slope1 * x + intercept1

    return x,y


# 교차점일 경우 사잇각 계산 
def calculate_intersection_angle(x1, y1, x2, y2, x3, y3):

    # 벡터 AB와 BC 계산
    ABx = x2 - x1
    ABy = y2 - y1
    BCx = x3 - x2
    BCy = y3 - y2
    
    angle_radians = math.atan2(ABx * BCy - ABy * BCx, ABx * BCx + ABy * BCy)
    angle_degrees = math.degrees(angle_radians)
    # 각도를 0-360도 범위로 변환
    if angle_degrees < 0:
        angle_degrees += 360
    return int(angle_degrees) - 180

def adjust_line_angle(frame, lane_lines, steering_ep):
    """
    감지된 차선들에 기반하여 조향각을 조정

    매개변수:
        frame : 현재 프레임/이미지
        lane_lines (list): 감지된 차선들의 리스트, 각 차선은 점 [x1, y1, x2, y2]의 리스트
        steering_ep (tuple): 조향 벡터의 끝점 (x1, y1, x2, y2)
        angle_prev_data (float): 이전 조향각 데이터
        one_lane_turn (float): 하나의 차선 전환에 대한 조향각

    반환값:
        float: 조정된 조향각
    """
    x1, y1, x2, y2 = steering_ep
    intersection_point = 0
    try:
        if len(lane_lines) == 1:
            if lane_lines[0][0][0] < frame.shape[1] // 2:
                intersection_point = ie(x1, y1, x2, y2, lane_lines[0][0][0], lane_lines[0][0][1]) * ie(x1, y1, x2, y2, lane_lines[0][0][2], lane_lines[0][0][3])
                if intersection_point == -1:
                    ix,iy = ip(x1,y1,x2,y2,lane_lines[0][0][0],lane_lines[0][0][1],lane_lines[0][0][2],lane_lines[0][0][3])
                    max_turn_angle = calculate_intersection_angle(lane_lines[0][0][2],lane_lines[0][0][3],ix,iy,ix,lane_lines[0][0][3])
                    return max_turn_angle
            elif lane_lines[0][0][0] > frame.shape[1] // 2:
                intersection_point = ie(x1, y1, x2, y2, lane_lines[0][0][0], lane_lines[0][0][1]) * ie(x1, y1, x2, y2, lane_lines[0][0][2], lane_lines[0][0][3])
                if intersection_point == -1:
                    ix,iy = ip(x1,y1,x2,y2,lane_lines[0][0][0],lane_lines[0][0][1],lane_lines[0][0][2],lane_lines[0][0][3])
                    max_turn_angle = calculate_intersection_angle(lane_lines[0][0][2],lane_lines[0][0][3],ix,iy,ix,lane_lines[0][0][3])
                    return max_turn_angle
        elif len(lane_lines) == 2:
            if lane_lines[0][0][0] < frame.shape[1] // 2:
                intersection_point = ie(x1, y1, x2, y2, lane_lines[0][0][0], lane_lines[0][0][1]) * ie(x1, y1, x2, y2, lane_lines[0][0][2], lane_lines[0][0][3])
                if intersection_point == -1:
                    ix,iy = ip(x1,y1,x2,y2,lane_lines[0][0][0],lane_lines[0][0][1],lane_lines[0][0][2],lane_lines[0][0][3])
                    max_turn_angle = calculate_intersection_angle(lane_lines[0][0][2],lane_lines[0][0][3],ix,iy,ix,lane_lines[0][0][3])     
                    return max_turn_angle
            if lane_lines[1][0][0] > frame.shape[1] // 2:
                intersection_point = ie(x1, y1, x2, y2, lane_lines[1][0][0], lane_lines[1][0][1]) * ie(x1, y1, x2, y2, lane_lines[1][0][2], lane_lines[1][0][3])
                if intersection_point == -1:
                    ix,iy = ip(x1,y1,x2,y2,lane_lines[1][0][0],lane_lines[1][0][1],lane_lines[1][0][2],lane_lines[1][0][3])
                    max_turn_angle = calculate_intersection_angle(lane_lines[1][0][2],lane_lines[1][0][3],ix,iy,ix,lane_lines[1][0][3])
                    return max_turn_angle
        else:
            return
    except:
        pass       

def angle_publish(angle):
    global angle_pub
    if angle_pub is None:
        angle_pub = rospy.Publisher('/angle', Float32, queue_size=5)
    
    rate = rospy.Rate(10)  # 10hz
    if not rospy.is_shutdown():
        rospy.loginfo(float(angle))
        angle_pub.publish(float(angle))


def standard_angle(angle):
    # print(angle)
    if 0 < angle <= 90 - max_angle:
        x = max_angle
    elif 90 - max_angle < angle <= 90 + max_angle:
        x = 90 - angle
    elif 90 + max_angle <= angle < 181:
        x = -max_angle
    else:
        x = 0
    return x

# up클래스 받았을때 조향각 계산 
def calculate_up_angle(x1, y1, x2, y2, x3, y3):
    # 벡터 AB와 BC 계산
    ABx = x2 - x1
    ABy = y2 - y1
    BCx = x3 - x2
    BCy = y3 - y2
    
    angle_radians = math.atan2(ABx * BCy - ABy * BCx, ABx * BCx + ABy * BCy)
    angle_degrees = math.degrees(angle_radians)
    
    # 각도를 0-360도 범위로 변환
    if angle_degrees < 0:
        angle_degrees += 360
    return int(angle_degrees - 90)


def slope_to_angle(slope):
    # 역탄젠트 함수로 라디안 값을 구합니다.
    radian = math.atan(slope)
    # 라디안 값을 도로 변환합니다.
    degree = math.degrees(radian)
    return degree

def heading_line_control(frame, steering_angle, lane_lines, handle_color=(0, 0, 255), line_color=(0, 255, 0), line_width=8):
    global label_coordinate, Turn_Flag, prev_turn_angle
    height, width, _ = frame.shape

    standard_slope_angle = 90 - slope_to_angle(min_slope)

    if label_coordinate: # up_클래스일 경우
        x1 = int(width / 2)
        y1 = height
        x_center, y_center = label_coordinate[0]
        up_steering_angle = calculate_up_angle(int(width / 2),y_center, int(width / 2),height, x_center, y_center)
        transform_angle = standard_angle(up_steering_angle)
        angle_publish(transform_angle)
        cv2.line(frame, (x1, y1), (x_center, y_center), handle_color, line_width)
    else:
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)
        adjust_data = adjust_line_angle(frame, lane_lines, [x1, y1, x2, y2])
        try:
            transform_angle = 0
            if adjust_data:
                Turn_Flag = True
                if adjust_data < 0:
                    transform_angle = adjust_line_angle(frame, lane_lines, [x1, y1, x2, y2]) * (max_angle / standard_slope_angle) - 0.5
                    prev_turn_angle = transform_angle 
                else:  
                    transform_angle = adjust_line_angle(frame, lane_lines, [x1, y1, x2, y2]) * (max_angle / standard_slope_angle) + 0.5
                    prev_turn_angle = transform_angle
            elif Turn_Flag == True:
                if len(lane_lines) == 2:
                    transform_angle = standard_angle(steering_angle)
                    Turn_Flag = False
                else:
                    transform_angle = prev_turn_angle
            else:
                transform_angle = standard_angle(steering_angle)
            angle_publish(transform_angle)
        except rospy.ROSInterruptException:
            pass
    
        cv2.line(frame, (x1, y1), (x2, y2), handle_color, line_width)
    #label_coordinate 초기화
    if label_coordinate: label_coordinate = []

#####################################################yolov5##############################################
def yolo_publish(objects):
    global object_pub
    if object_pub is None:
        object_pub = rospy.Publisher('/object', String, queue_size=5)
    
    rate = rospy.Rate(10)  # 10hz
    if not rospy.is_shutdown():
        rospy.loginfo(objects)
        object_pub.publish(objects)

def yolo_traffic(frame, stride, names):
    # global _class
    img = letterbox(frame, stride=stride, auto=True)[0]
    img = img.transpose((2, 0, 1))[::-1]
    img = np.ascontiguousarray(img)

    img = torch.from_numpy(img).to(device)
    img = img.half() if model.fp16 else img.float()
    img /= 255
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    with torch.no_grad():
        pred = model(img, augment=False, visualize=False)
        pred = non_max_suppression(pred, 0.25, 0.45, None, False, max_det=1000)

    for det in pred: #up 4
        if len(det):
            tmp = []
            det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], frame.shape).round()
            for *xyxy, conf, cls in reversed(det):
                x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                label = f'{names[int(cls)]} {conf:.2f}'
                yolo_publish(names[int(cls)])
                x_center = (x1 + x2) // 2
                y_center = (y1 + y2) // 2
                if names[int(cls)] == 'up':
                    # print(frame[0])
                    if conf > 0.5 and frame.shape[0] - 150 > y2:
                        cv2.line(frame, (0, frame.shape[0] - 150), (frame.shape[1], frame.shape[0] - 150), (0, 0, 255), 2)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(frame, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        tmp.append([x_center,y_center,conf])
                else:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if tmp:
                for _ in range(len(tmp)):
                    for j in range(len(tmp)-1):
                        if tmp[j][2] < tmp[j+1][2]:
                            tmp[j],tmp[j+1] = tmp[j+1],tmp[j]
                best_x_center,best_y_center = tmp[0][0],tmp[0][1]
                label_coordinate.append([best_x_center,best_y_center])
        else:
            yolo_publish('None')

#####################################################yolov5#############################################

def adjust_brightness_contrast(frame, alpha=1.0, beta=0):
    adjusted = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
    return adjusted

if __name__ == "__main__":
    rospy.init_node('lane_detect_node', anonymous=True)
    # cap = cv2.VideoCapture('tinywow_IMG_1590_37023169.mp4')
    # cap = cv2.VideoCapture(5)
    cap = cv2.VideoCapture(2)
    model = DetectMultiBackend(weights, device=device, dnn=False, fp16=False)
    stride, names, pt = model.stride, model.names, model.pt

    while cap.isOpened():
        if cv2.waitKey(1) == ord('q'):
            break

        ret, frame = cap.read()
        if not ret:
            break
        yolo_traffic(frame, stride, names)
        # print(alpha)
        if alpha != 1.0 or beta != 0:
            frame = adjust_brightness_contrast(frame, alpha, beta)
        edges = detect_edges(frame)
        masked_image = region_of_interest(edges)
        line_segments = detect_line_segments(masked_image)
        lane_lines = average_slope_intercept(frame, line_segments)
        draw_lines(frame, lane_lines)
        steering_angle = compute_steering_angle(frame, lane_lines)
        stabilized_steering_angle = stabilize_steering_angle(90, steering_angle, len(lane_lines))
        heading_line_control(frame, stabilized_steering_angle, lane_lines)
        
        cv2.imshow('Detected Lanes', frame)
    
    cap.release()
    cv2.destroyAllWindows()
