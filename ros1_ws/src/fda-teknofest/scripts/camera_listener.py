#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
from detect_target import DetectTarget
from costmap import CostMap
import numpy as np
import cv2

# Modelin Adresi
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, '..', 'best.pt')
model_path = os.path.normpath(model_path)
model = YOLO(model_path)
detectObj = DetectTarget()
costmapObj = CostMap(fx=1739.1304,fy=1739.1304,cx=728.0,cy=544.0)
best_area = 0
best_center = None
best_mask = None
best_costmap_location = None

#! BU DEĞERLER HER FRAME DE GÜNCELLENMELİ
currentRoll = 0
currentPitch = 0
currentYaw = 0
currentUav_x = 0
currentUav_y = 0
currentAltitude = 10

def callback(msg):
    global best_area, best_center, best_mask, best_costmap_location
    #! YOLO'ya görüntü verilmeden önce görüntü undistort edilmeli !!!
    
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # H matrisi
    H = costmapObj.calculate_H(currentRoll, currentPitch, currentYaw, currentUav_x, currentUav_y, currentAltitude)
      
    # Target noktanın bulunması, tüm frameler arasında en büyük alanlı şeklin hedef olarak alınması
    h_mask, center, current_area = detectObj.detect_target(cv_image)
    
    if H is not None and center[0] is not None:

        point = np.array(
            [[[center[0], center[1]]]],
            dtype=np.float32
        )

        map_point = cv2.perspectiveTransform(point, H)

        map_x, map_y = map_point[0, 0]

        col = int(round(map_x))
        row = int(round(map_y))

        if 0 <= row < costmapObj.SIZE and 0 <= col < costmapObj.SIZE:

            if current_area > best_area:
                best_area = current_area
                best_center = center
                best_mask = h_mask.copy()
                best_costmap_location = (row, col)
        
    
    #! Verbose production da kapat
    results = model(source=cv_image, imgsz=640, verbose=True)
    
    # YOLO'nun çizdiği renkli frame
    annotated_frame = results[0].plot()
    
    # YOLO'nun döndürdüğü maske
    mask = results[0].semantic_mask.data.cpu().numpy() # (1088, 1456) uint8
    
    # Maliyet haritası güncellemesi
    costmapObj.update(mask=mask, H=H)
    
    # YOLO'nun çizdiği renkli framein yayınlanması, sadece test için
    out_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding='passthrough')
    out_msg.encoding = 'bgr8'
    out_msg.header = msg.header
    pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('camera_listener')
    bridge = CvBridge()
    pub = rospy.Publisher('camera/yolo_frame', Image, queue_size=1)
    rospy.Subscriber('/camera/image_raw', Image, callback)
    rospy.spin()
