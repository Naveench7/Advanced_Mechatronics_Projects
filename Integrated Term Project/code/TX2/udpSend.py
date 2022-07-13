import socket
import rospy
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class udpRecv:
    def __init__(self):
        #self.ip = "192.168.73.210"
        self.ip = "192.168.73.38"
        self.port = 14687
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_sub = rospy.Subscriber("/marker_detection", Point, self.marker_callback)
        self.int_sub = rospy.Subscriber("/intersection", Bool, self.intersection_callback)
        time.sleep(2)
        print("Sending Start")
        message = "Start"
        self.sock.sendto(message.encode("utf-8"), (self.ip, self.port))
        rospy.spin()


    def marker_callback(self, msg):
        print("Sending marker")
        message = ""
        if msg.x < 10:
            message = "Friend found"
        elif msg.x > 10:
            message = "Enemy found"
        self.sock.sendto(message.encode("utf-8"), (self.ip, self.port))

    def intersection_callback(self, msg):
        print("sending intersection")
        message = "Intersection"
        self.sock.sendto(message.encode("utf-8"), (self.ip, self.port))


rospy.init_node("udp_recv")
ur = udpRecv()



