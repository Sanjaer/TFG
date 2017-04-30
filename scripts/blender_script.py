#      blender_script.py
# Developed by Pablo San José                 
#     License CC-BY-SA    ***/

import bge
import GameLogic as GL
import math
import json
import socket
import os
import sys
import threading
import time
import logging
from datetime import datetime

##### Thread #####

# Main function for moverZ_threads
# thread_bone -> object channel (bone)
def moverZ(thread_bone, owner, socket):
    GL.logging.info(threading.current_thread().name + "START: " + str(time.time()))
    setupLoggerFile("INFO", "a")

    data = -1

    try:
        data, addr = socket.recvfrom(40)
    except:
        pass

    if data == -1:
        GL.logging.debug("No data yet!")
        # print("no data")
    else:
        GL.logging.info("Data received: " + str(data))
        applyRotationZ(thread_bone, data)
        owner.update()

    GL.logging.info(threading.current_thread().name + "FINISH: " + str(time.time()))


# Rotation function for moverZ_threads
def applyRotationZ(bone, data):
    rot_z = GL.lut1[float(data)]
    bone.rotation_mode = GL.ROT_MODE_ZYX
    bone.rotation_euler=[0,0,rot_z] #BL_ArmatureChannel.rotation_euler
    #print("Bone moved: ", bone.name)


def moverX(owner, socket):
    setupLoggerFile("INFO", "a")
    GL.logging.info(threading.current_thread().name + "START: " + str(time.time()))

    data = ""

    try:
        data, addr = socket.recvfrom(40)
    except:
        pass

    # print with showing off purpouse only
    if len(data) == 0:
        print("no data")
        GL.logging.debug("No data yet!")
    else:
        print("data -> ", data)
        data = str(data,'utf-8')
        GL.logging.info("Data received: " + data + threading.current_thread().name + str(time.time()))
        applyRotationX(data)
        owner.update()
    GL.logging.info(threading.current_thread().name + "FINISH: " + str(time.time()))


# Rotation function for moverX_threads
def applyRotationX(data):
    
    data_ds = json.loads(data)
    print("data_ds -> ", data_ds)

    for bone in GL.x_bones:
        bone.rotation_mode = GL.ROT_MODE_ZYX
    
    rot_thumb = GL.lut2[round(data_ds[0], 4)] # Pulgar
    GL.x_bones[4].rotation_euler=[rot_thumb, 0, 0] #BL_ArmatureChannel.rotation_euler

    v1 = GL.lut2[round(data_ds[1], 4)]
    v2 = GL.lut2[round(data_ds[2], 4)]
    rot4 = GL.lut2[round(data_ds[3], 4)]

    rot1 = v1 - v2
    if rot1 > 0:
        rot2 = v1 - rot1
        rot3 = v2
        GL.x_bones[0].rotation_euler = [rot2/4, 0, 0] #BL_ArmatureChannel.rotation_euler
        GL.x_bones[1].rotation_euler = [rot1/4, 0, 0] #BL_ArmatureChannel.rotation_euler
        GL.x_bones[2].rotation_euler = [(rot1 + rot3)/4, 0, 0] #BL_ArmatureChannel.rotation_euler
        GL.x_bones[3].rotation_euler = [(rot1 + rot3 + rot4)/4, 0, 0] #BL_ArmatureChannel.rotation_euler


    else:
        rot2 = v2-rot1
        rot3 = v1
        GL.x_bones[0].rotation_euler = [(rot1 + rot3)/4, 0, 0] #BL_ArmatureChannel.rotation_euler
        GL.x_bones[1].rotation_euler = [rot1/4, 0, 0] #BL_ArmatureChannel.rotation_euler
        GL.x_bones[2].rotation_euler = [rot2/4, 0, 0] #BL_ArmatureChannel.rotation_euler
        GL.x_bones[3].rotation_euler = [(rot2 + rot4)/4, 0, 0] #BL_ArmatureChannel.rotation_euler
        
##### End Thread #####

##### Global Variables #####
nl_fingers = ["indice1", "indice2",
                "corazon1", "corazon2", 
                "anular1", "anular2", 
                "menique1", "menique2",
                "pulgar1", "pulgar2"]

nl_sep = ["pulgarindice",  "indicecorazon", 
            "corazonanular", "anularmenique"]

not_allowed = ["hand.wrist", "index.d_phalanx", "middle.d_phalanx", "ring.d_phalanx", "pinky.d_phalanx", "thumb.methacarpal"]

x_bones = ["index.p_phalanx", "middle.p_phalanx", "ring.p_phalanx", "pinky.p_phalanx", "thumb.methacarpal"]

num_sockets = 11
##### End Global Variables #####

##### StartUp Functions #####
def setupLoggerFile(level, mode):
    logger_file = './log/udp_blenthread_' + level + '_' + str(time.strftime("%Y%m%d")) + '.log'

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
        
    GL.logging = logging


def setupBonesLists(hand):
    bones_list = [bone for bone in hand.channels if bone.name not in not_allowed]
    GL.num_bones = len(bones_list)

    x_bones = [bone for bone in hand.channels if bone.name in x_bones]

    GL.bones_list = bones_list
    GL.x_bones = x_bones


def calibration():
    address = '/tmp/'
    addr_cal = []
    addr_cal.append(address + 'calibration_max')
    addr_cal.append(address + 'calibration_min')

    sock_cal = []
    sock_cal.append(socket.socket(socket.AF_UNIX, socket.SOCK_STREAM))
    sock_cal.append(socket.socket(socket.AF_UNIX, socket.SOCK_STREAM))
   
    data_max = []

    try:
        sock_cal[0].connect(addr_cal[0])
        sock_cal[1].connect(addr_cal[1])
    except socket.error:
        GL.logging.error("Error when connecting to calibration sockets" + str(time.time()))
        raise

    try:
        data = sock_cal[0].recv(168).decode()
        logging.info("Calibration max: " + data)
        data_max = json.loads(data)
        data = sock_cal[1].recv(168).decode()
        data_min = json.loads(data)
        logging.info("Calibration max: " + data)
    except:
        GL.logging.error("Error when reading from calibration sockets" + str(time.time()))
        raise

    for sock in sock_cal:
        sock.close()


def setupSockets():
    address = '/tmp/'

    # Sockets setup
    s = [socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) for i in range(num_sockets)]
    socket_address = [address + nl_fingers[i] for i in range(num_sockets-1)]
    socket_address.append(address + "expansion")

    for s_address in socket_address:
        # Make sure the socket does not already exist
        try:
            os.unlink(s_address) # Remove (delete) the file path
        except OSError:
            if os.path.exists(s_address):
                GL.logging.warning("OSError: " + s_address)
                raise

    for i in range(num_sockets):
        s[i].setblocking(0)
        s[i].bind(socket_address[i])

    GL.s=s

    
def fill_luts():
    lut1 = {x/10000 : round(-1*x/10000 * math.radians(90), 4) for x in range (0, 10001)}
    GL.lut1 = lut1
    lut2 = {x/10000 : round(1*x/10000 * math.radians(45), 4) for x in range (0, 10001)}
    GL.lut2 = lut2


##### End StartUp Functions #####


if __name__ == '__main__':

    # Get the controller of the scene, so we can manipulate it
    cont = bge.logic.getCurrentController()
    hand = cont.owner
    
    # Block of code executed only once
    if not hand['StartUp']:
        # Setup logger file
        setupLoggerFile("INFO", "w") #DEBUG - INFO - WARNING
    	
        GL.logging.debug("StartUp setup")
        GL.logging.info("Only once block START: " + str(time.time()))

        #Setup bones lists
        try:
            setupBonesLists(hand)
        except:
            GL.logging.error("Error extracting bones' names from the scene")
        
        # Calibration (Optional)
        try:
            calibration()
        except:
            GL.logging.error("Error receiving calibration")

        #Filling of the LookUp Tables
        try:
            fill_luts()
        except:
            GL.logging.error("Error filling LUT")

        # Setup of all the sockets
        try:
            setupSockets()
        except:
            GL.logging.error("Error creating sockets")
 
        hand['StartUp'] = True
        GL.logging.info("Only once block FINISH: " + str(time.time()))
    
    else:

        GL.logging.info("Threads' creation START: " + str(time.time()))
        # Creation of the threads (ALL OK)
        try: 
            mover_threads = [threading.Thread(name=GL.bones_list[i], target=moverZ, args=[GL.bones_list[i], hand, GL.s[i]]) for i in range(GL.num_bones)]
            mover_threads.append(threading.Thread(name="btwFingers", target=moverX, args=[hand, GL.s[-1]]))
        except:
            GL.logging.error("Error creating threads")

        for t in mover_threads:
            try:
                t.start()
            except:
                GL.logging.error("Error starting threads")

        for t in mover_threads:
            try:
                t.join()
            except:
                GL.logging.error("Error joining threads for Z")
        
        GL.logging.info("Threads' creation FINISH: " + str(time.time()))