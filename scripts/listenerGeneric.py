#      main_sub.py
# Developed by Pablo San Jose                 
#     License CC-BY-SA    ***/

#!/usr/bin/python

from multiprocessing import Process
import rospy
import json
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import socket
import os
import sys
import threading
from array import array
import logging
import time

#########     Global variables     #########

address = '/tmp/' # Path for the sockets

# List of chat's names for flexion sensors
nl_fingers = ["indice1", "indice2",
                "corazon1", "corazon2", 
                "anular1", "anular2", 
                "menique1", "menique2",
                "pulgar1", "pulgar2"]

# List of chat's names for separation sensors
nl_sep = ["pulgarindice",  "indicecorazon", 
            "corazonanular", "anularmenique"]


#########     End Global variables     #########

#########     Listeners' classes     #########

class listenerGeneric():

    def __init__(self, name, id_p, socket=None):
        self.name = name
        self.id = id_p
        self.address = address + name 
        if socket is None:
            socket = ""
        self.socket = socket

    def run(self):
        logging.info(self.name + "START"+ str(time.time()))
        # Starts ROS communication
        self.listen()
        logging.info(self.name + "FINISH"+ str(time.time()))

class listenerFingers(listenerGeneric):

    def listen(self):
        logging.info(self.name + " listen " + "START"+ str(time.time()))

        node_name = self.name + "_listener"
        rospy.init_node(node_name, anonymous=True)

        rospy.Subscriber(self.name, Float64, self.callback)
        logging.info(self.name + " spin " + str(time.time()))
        rospy.spin()

    def callback(self, data):
        logging.info(self.name + " callback " + "START"+ str(time.time()))
        data_send = str(round(data.data, 4))
        rospy.loginfo(rospy.get_caller_id() + "I heard %s i'll send %s", data.data, data_send)
        # Sends the info over the UDP socket
        self.socket.sendto(data_send, self.address)
        logging.info(self.name + " callback " + "FINISH"+ str(time.time()))


class listenerExp(listenerGeneric):

    def listen(self):
        logging.info(self.name + " listen " + "START"+ str(time.time()))
        node_name = self.name + "_listener"
        rospy.init_node(node_name, anonymous=True)

        rospy.Subscriber("expansion", Float64MultiArray, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self, data):
        logging.info(self.name + " callback " + "START"+ str(time.time()))
        ds_serialized = json.dumps(data.data)
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s i'll send %s", data.data, ds_serialized)
        self.socket.sendto(ds_serialized, self.address)
        logging.info(self.name + " callback " + "FINISH"+ str(time.time()))


# Class designed to read from the calibration publishers and transmit
# the data obtained through TCP it's sockets
class listenerCalibration(listenerGeneric):

    lock = threading.Lock()

    def listen(self):
        server_thread = threading.Thread(name=(self.name + "_server"), target=self.server, args=[])
        server_thread.start()

        node_name = self.name + "_listener"
        rospy.init_node(node_name, anonymous=True)
        rospy.Subscriber(self.name, Float64MultiArray, self.callback)
        
        rospy.spin()

    def server(self):
        # file_name = self.name + '.txt'
        if self.name == "calibration_max":
            file_name = "calibration_max.txt"
        else:
            file_name = "calibration_min.txt"

        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)# Configure sock_stream
        sock_addr = address + self.name

        try:
            os.unlink(sock_addr) # Remove (delete) the file path
        except OSError:
            if os.path.exists(sock_addr):
                logging.warning("OSError: " + sock_addr)
                raise
        s.bind(sock_addr)
        s.listen(1)
        while(True):
            connection, client_address = s.accept()
            # Get lock
            self.lock.acquire()
            try:
                with open(file_name, 'r') as input_cal:
                    cal_array = array('f')
                    cal_array.fromstring(input_cal.read())
                    data_send = cal_array.tolist()
                
            finally:
                self.lock.release()

            ds_serialized = json.dumps(data_send)

            connection.sendall(ds_serialized.encode())
            connection.close()

    def callback(self, data):
        file_name = self.name + '.txt'
        ds_deserialized = data.data
        
        self.lock.acquire()
        try:
            with open(file_name, 'wb') as write_cal:
                float_array = array('f', ds_deserialized)
                float_array.tofile(write_cal)
        finally:
            self.lock.release()

#########     Listeners' classes     #########

#########    Listeners' launchers    #########

def launchListenerFingers(name, id_p):
    setupLoggerFile('main_sub', 'INFO', 'a')
    logging.info("startListenerFingers START" + str(time.time()))
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) 
    listener_obj = listenerFingers(name, id_p, s)
    listener_obj.run()
    logging.info("startListenerFingers FINISH" + str(time.time()))


def launchListenerExp(name, id_p):
    setupLoggerFile('main_sub', 'ERROR', 'a')
    logging.info("launchListenerExp START" + str(time.time()))
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    listener_obj = listenerExp("expansion", id_p, s)
    listener_obj.run()
    logging.info("launchListenerExp FINISH" + str(time.time()))


def launchListenerCalibration(name, id_p):
    setupLoggerFile('main_sub', 'ERROR', 'a')
    logging.info("launchListenerCalibration START" + str(time.time()))
    listener_obj = listenerCalibration(name, id_p)
    listener_obj.run()
    logging.info("launchListenerCalibration FINISH" + str(time.time()))

#########    Listeners' launchers    #########

#########            Logger          #########

def setupLoggerFile(name, level, mode):
    logger_file = './log/' + name + '_' + level + '_' + str(time.strftime("%Y%m%d")) + '.log'
    print("Logger file: " + logger_file)

    if level is "DEBUG":
        logging.basicConfig(format='%(asctime)s %(message)s',
                datefmt='%m/%d/%Y %I:%M:%S %p',
                filename=logger_file,
                filemode=mode,
                level=logging.DEBUG)
    elif level is "WARNING":
        logging.basicConfig(format='%(asctime)s %(message)s',
                datefmt='%m/%d/%Y %I:%M:%S %p',
                filename=logger_file,
                filemode=mode,
                level=logging.WARNING)
    else:
        logging.basicConfig(format='%(asctime)s %(message)s',
                datefmt='%m/%d/%Y %I:%M:%S %p',
                filename=logger_file,
                filemode=mode,
                level=logging.INFO)

#########            Logger          #########
