#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from std_msgs.msg import String

def process_frame(frame, pub):
    gray_frame = frame.copy()
    gray_frame[:frame.shape[0]//2,:] = [0,0,0]
    gray = cv2.cvtColor(gray_frame, cv2.COLOR_BGR2GRAY)
    ####################################################
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 가우시안 블러를 사용하여 노이즈 감소
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 이진화를 사용하여 흰색 선 검출
    _, binary = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # 캐니 엣지을 통한 경계 검출
    edges = cv2.Canny(binary, 200, 400)

    # 경계선 확장 및 경계선 검출
    dilated = cv2.dilate(edges, None, iterations=2)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def is_crosswalk(contours):
        rects = [cv2.boundingRect(c) for c in contours if cv2.contourArea(c) > 500]
        rects = sorted(rects, key=lambda r: r[1])  # y 좌표 기준으로 정렬

        if len(rects) < 3:
            return False

        for i in range(1, len(rects)):
            if abs(rects[i][1] - rects[i-1][1]) > rects[i][3] * 1.5:
                return False
        return True

    cv2.line(frame, (0, frame.shape[0] - 50), (frame.shape[1], frame.shape[0] - 50), (255, 0, 0), 2)
    send = []

    for contour in contours:
        if cv2.contourArea(contour) > 6000:  # 면적 임계값
            rect = cv2.boundingRect(contour)
            x, y, w, h = rect
            aspect_ratio = w / h
            if aspect_ratio > 5:  # 가로로 긴 형태 검출
                cv2.putText(frame, 'Crosswalk', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                if y + h >= frame.shape[0] - 50: #and y + h <= frame.shape[0]:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    send.append('stop')
                else:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    send.append('decrease')

                 
    if 'stop' in send:
        pub.publish("stop")
        return frame
    elif 'decrease' in send:
        pub.publish("decrease")
        return frame
    else:
        pub.publish("go")
        return frame

def main():
    rospy.init_node('crosswalk_detector', anonymous=True)
    pub = rospy.Publisher('/crosswalk', String, queue_size=5)
    
    #cap = cv2.VideoCapture('tinywow_IMG_1590_37023169.mp4')
    # ls /dev/video*
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            break
            
        processed_frame = process_frame(frame, pub)
        cv2.imshow('Detected Crosswalk', processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
