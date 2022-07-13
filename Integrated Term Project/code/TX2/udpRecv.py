import rospy
import socket

from std_msgs.msg import Bool

class udpServer:
    def __init__(self):
        self.ip = "0.0.0.0" 
        self.port = 15689
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.start_publisher = rospy.Publisher("/start_command", Bool, queue_size=1)
        print("starting")
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            data, addr = self.sock.recvfrom(1024)
            print(data)
            msg = Bool()
            msg.data = True
            self.start_publisher.publish(msg)

rospy.init_node("starter")
udprecv = udpServer()

