import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class ArucoDetector:
    def __init__(self):
        self.aruco_key = getattr(cv2.aruco, 'DICT_6X6_250')
        self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_key)
        self.aruco_param = cv2.aruco.DetectorParameters_create()

        self.marker_publisher = rospy.Publisher("/marker_detection", Point, queue_size=1)
        self.marker_msg = Point()

        self.detected_ids = []
        self.marker_size = 2.54 # In cm
        self.cam1K = np.array([[385.09765625, 0.0, 320.96099853515625], [0.0, 385.09765625, 239.1181640625], [0.0, 0.0, 1.0]])
        self.cam2K = [[385.09765625, 0.0, 320.96099853515625], [0.0, 385.09765625, 239.1181640625], [0.0, 0.0, 1.0]]
        self.dist = np.array([0., 0., 0., 0.])

        self.image1_sub = rospy.Subscriber("/camera/infra1/image_rect_raw", Image, self.cam1_cb)
        self.image2_sub = rospy.Subscriber("/camera/infra2/image_rect_raw", Image, self.cam2_cb)
        rospy.spin()
    
    def cam1_cb(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) #self.bridge.imgmsg_to_cv2(msg, "mono8")
        (bboxs, ids, rejected) = cv2.aruco.detectMarkers(image, self.aruco_dict, parameters = self.aruco_param)

        if ids is None:
            return

        rvecs, tvecs, trash = cv2.aruco.estimatePoseSingleMarkers(bboxs, self.marker_size, self.cam1K, self.dist)

        for idx, id in enumerate(ids):
            norm = math.sqrt((tvecs[idx][0][0]*tvecs[idx][0][0]) + (tvecs[idx][0][1]*tvecs[idx][0][1]) + (tvecs[idx][0][2]*tvecs[idx][0][2])) 
            if id in self.detected_ids or norm > 25.:
                return
            
            
            self.detected_ids.append(id)
            self.marker_msg.x = id
            sum = 0
            for i in range(4):
                sum += bboxs[idx][0][i][0]
            sum /= 4.0
            print(sum)
            if(sum < 400):
                print("LEFT")
                self.marker_msg.y = -1
            else:
                print("RIGHT")
                self.marker_msg.y = 1
            self.marker_msg.z = norm
            self.marker_publisher.publish(self.marker_msg)
        return
    
    def cam2_cb(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) #self.bridge.imgmsg_to_cv2(msg, "mono8")
        (bboxs, ids, rejected) = cv2.aruco.detectMarkers(image, self.aruco_dict, parameters = self.aruco_param)

        if ids is None:
            return

        rvecs, tvecs, trash = cv2.aruco.estimatePoseSingleMarkers(bboxs, self.marker_size, self.cam1K, self.dist)

        for idx, id in enumerate(ids):
            norm = math.sqrt((tvecs[idx][0][0]*tvecs[idx][0][0]) + (tvecs[idx][0][1]*tvecs[idx][0][1]) + (tvecs[idx][0][2]*tvecs[idx][0][2])) 
            if id in self.detected_ids or norm > 25.:
                return
            
            
            self.detected_ids.append(id)
            self.marker_msg.x = id
            sum = 0
            for i in range(4):
                sum += bboxs[idx][0][i][0]
            sum /= 4.0
            print(sum)
            if(sum < 400):
                print("LEFT")
                self.marker_msg.y = -1
            else:
                print("RIGHT")
                self.marker_msg.y = 1
            self.marker_msg.z = norm
            self.marker_publisher.publish(self.marker_msg)
        return

rospy.init_node("TestRS")
ad = ArucoDetector()
