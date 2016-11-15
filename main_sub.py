# Script in /home/proyecto/ros-tfg/catkin_ws/src/tfg/scripts

from multiprocessing import Process
import rospy
import json
from std_msgs.msg import Float64
from std_msgs.msg import String
import time
import socket
import os
import sys

address = '/tmp/'

packet_model = {'/pulgarindice': '','/indicecorazon': '', 
            '/corazonanular': '', '/anularmenique': ''}

class listener_class_fingers():

    def __init__(self, name, id_p, socket):
        self.name = name
        self.id = id_p
        self.socket = socket
        self.address = address + name 

    def run(self):
        logging.info(self.name + "START")
        # Starts ROS communication
        self.listen()
        #time.sleep(5)
        logging.info(self.name + "FINISH")

    def listen(self):
        logging.info(self.name + " listen " + "START")

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'talker' node so that multiple talkers can
        # run simultaneously.
        node_name = self.name + "_listener"
        rospy.init_node(node_name, anonymous=True)

        rospy.Subscriber(self.name, Float64, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        logging.info(self.name + " listen " + "FINISH") # Check this one<<<<<<<<<<<<<

    def callback(self, data):
        logging.info(self.name + " callback " + "START")
        data_send = str(round(data.data, 4))
        rospy.loginfo(rospy.get_caller_id() + "I heard %s i'll send %s", data.data, data_send)
        # Sends the info over the udp socket
        self.socket.sendto(data_send, self.address)
        logging.info(self.name + " callback " + "FINISH")


class listener_class_sep():

    def __init__(self, name, id_p, socket):
        self.name = name
        self.id = id_p
        self.socket = socket
        self.address = address + name

    def run(self):
        logging.info(self.name + "START")
        # Starts ROS communication
        self.listen()
        #time.sleep(5)
        logging.info(self.name + "FINISH")

    def listen(self):
        logging.info(self.name + " listen " + "START")
        node_name = self.name + "_listener"
        rospy.init_node(node_name, anonymous=True)

        rospy.Subscriber("expansion", String, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        logging.info(self.name + " listen " + "FINISH")

    def callback(self, data):
        logging.info(self.name + " callback " + "START")
        # self.data_send = json.dumps(data)
        data_send = data.data
        print(data_send)
        self.socket.sendto(data_send, self.address)
        logging.info(self.name + " callback " + "FINISH") # Check this one<<<<<<<<<<<<<



def listener_func(name, id_p):
    logging.info("listener_func START" + str(time.time()))
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    listener_obj = listener_class_fingers(name, id_p, s)
    listener_obj.run()
    logging.info("listener_func FINISH" + str(time.time()))


def listener_sep(name, id_p):
    logging.info("listener_sep START" + str(time.time()))
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    listener_obj = listener_class_sep("expansion", id_p, s)
    listener_obj.run()
    logging.info("listener_sep FINISH" + str(time.time()))


nl_fingers = ["indice1", "indice2",
                "corazon1", "corazon2", 
                "anular1", "anular2", 
                "menique1", "menique2",
                "pulgar1", "pulgar2"]

nl_sep = ["pulgarindice",  "indicecorazon", 
            "corazonanular", "anularmenique"]


if __name__ == '__main__':

    #logger_file = "udp_blenthread" + str(time.time()) + DATE + ".log"
    logging.basicConfig(filename='debugger.log',level=logging.DEBUG)
    #logging.basicConfig(filename='udp_blenthread.log',level=logging.INFO) # Recomended

    #Calibration
    # s_cal = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    # cal = listener_class("calibration", 100, s_cal)

    logging.info("Processes creation FINISH" + str(time.time()))

    p=[]
    # Code to launch all processes together
    p = [Process(target=listener_func, args=(nl_fingers[i], i)) for i in range(len(nl_fingers))]
    p.append(Process(target=listener_sep, args=("sep", 11)))

    logging.info("Processes creation FINISH" + str(time.time()))


    logging.info("Processes starting START" + str(time.time()))
    # This must be separated in two different for loops
    for process in p:
        process.start()


    logging.info("Processes starting START" + str(time.time()))

    logging.info("Processes joining START" + str(time.time()))

    for process in p:
        process.join()


    logging.info("Processes joining FINISH" + str(time.time())) # Check out this one

