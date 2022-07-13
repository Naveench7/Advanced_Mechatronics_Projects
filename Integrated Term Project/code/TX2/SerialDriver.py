#! /usr/bin/env python3

import rospy
import serial
import time
import threading


from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class TeensySerialDriver:
    def __init__(self):
        self.seialPortLoc =  "/dev/ttyTHS2"
        self.serialPortBaud = 115200
        self.serPort = serial.Serial(self.seialPortLoc, self.serialPortBaud, timeout=0.5)
        self.intersection_pub = rospy.Publisher("/intersection", Bool, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacle", Bool, queue_size=1)
        self.intersection_msg = Bool()
        self.intersection_msg.data = True
        time.sleep(2)
        #msg_arduino = "H"
        #self.serPort.write(msg_arduino.encode("utf-8"))

        self.run_thread = threading.Thread(target = self.run)
        self.run_thread.start()

        self.marker_subscriber = rospy.Subscriber("/marker_detection", Point, self.marker_callback)
        self.start_subscriber = rospy.Subscriber("/start_command", Bool, self.start_callback)
        rospy.spin()

    def run(self):
        while not rospy.is_shutdown():
            data = self.serPort.readline().decode("utf-8")[:-2]
            # print(data)
            if data == "Intersection":
                print("Found Intersection")
                self.intersection_pub.publish(self.intersection_msg)
            if data == "Obstacle":
                print("Found obstacle")
                self.obstacle_pub.publish(self.intersection_msg)

    def marker_callback(self, msg):
        msg_arduino = "X"
        if msg.x < 10:
            msg_arduino += "F"
        else:
            msg_arduino += "E"
        
        if msg.y == 1:
            msg_arduino += "R"
        else:
            msg_arduino += "L"
        print(msg_arduino)
        self.serPort.write(msg_arduino.encode("utf-8"))

    def start_callback(self, msg):
        msg_arduino = "HHH"
        self.serPort.write(msg_arduino.encode("utf-8"))


rospy.init_node("serial_driver")
tsd = TeensySerialDriver()
