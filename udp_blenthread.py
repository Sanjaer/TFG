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
# refnode -> name of the socket of which must read
def moverZ(thread_bone, refnode, owner, socket):
    GL.logging.info(threading.current_thread().name + "START: " + str(time.time()))

    data = -1
    #ESTO SIGUE SIN FUNCIONAR
    # try:
    #     data, addr = socket.recvfrom(40)
    #     print("Data received!" + data)
    #     #applyRotationZ(thread_bone, data)
    #     #owner.update()
    # except:
    #     print("No data yet!")

    try:
        data, addr = socket.recvfrom(40)
    except:
        pass

    # print with showing off purpouse only
    if data == -1:
        GL.logging.info("No data yet!")
    else:
        rot = float(data)
        applyRotationZ(thread_bone, rot)
        owner.update()

    GL.logging.info(threading.current_thread().name + "FINISH: " + str(time.time()))

def moverX(thread_bones, ref_nodes, owner, socket):
    GL.logging.info(threading.current_thread().name + "START: " + str(time.time()))
    #print(threading.current_thread().name, "Starting")

    data = ""

    try:
        data, addr = socket.recvfrom(24)
    except:
        pass

     # print with showing off purpouse only
    if len(data) == 0:
        GL.logging.info("No data yet!")
    else:
        applyRotationX(data)
        owner.update()
    GL.logging.info(threading.current_thread().name + "FINISH: " + str(time.time()))
    #print(threading.current_thread().name, "Exiting")


# Rotation function for moverZ_threads
def applyRotationZ(bone, rot):
    rot = -100*rot
    print(rot)
    rot_z = math.radians(rot)
    bone.rotation_mode = GL.ROT_MODE_ZYX
    bone.rotation_euler=[0,0,rot_z] #BL_ArmatureChannel.rotation_euler
    #print("Bone moved: ", bone.name)

# Rotation function for moverX_threads
def applyRotationX(data):
    GL.logging.debug(float(data[:6]))
    GL.logging.debug(float(data[6:12]))
    GL.logging.debug(float(data[12:18]))
    GL.logging.debug(float(data[18:]))

    for bone in xBones:
        bone.rotation_mode = GL.ROT_MODE_ZYX
    
    rot_thumb = math.radians(float(data[:6])) # Pulgar
    xBone[4].rotation_euler=[rot_thumb, 0, 0] #BL_ArmatureChannel.rotation_euler

    v1 = float(data[6:12])
    v2 = float(data[12:18])
    rot4 = float(data[18:])

    rot1 = v1 - v2
    if rot1 > 0:
        rot2 = v1 - rot1
        rot3 = v2
        xBone[0].rotation_euler = [rot2, 0, 0] #BL_ArmatureChannel.rotation_euler
        xBone[1].rotation_euler = [rot1, 0, 0] #BL_ArmatureChannel.rotation_euler
        xBone[2].rotation_euler = [rot1 + rot3, 0, 0] #BL_ArmatureChannel.rotation_euler
        xBone[3].rotation_euler = [rot1 + rot3 + rot4, 0, 0] #BL_ArmatureChannel.rotation_euler


    else:
        rot2 = v2-rot1
        rot3 = v1
        xBone[0].rotation_euler = [rot1 + rot3, 0, 0] #BL_ArmatureChannel.rotation_euler
        xBone[1].rotation_euler = [rot1, 0, 0] #BL_ArmatureChannel.rotation_euler
        xBone[2].rotation_euler = [rot2, 0, 0] #BL_ArmatureChannel.rotation_euler
        xBone[3].rotation_euler = [rot2 + rot4, 0, 0] #BL_ArmatureChannel.rotation_euler
        
##### End Thread #####

##### Global Variables #####

address = '/tmp/'

nl_fingers = ["indice1", "indice2",
                "corazon1", "corazon2", 
                "anular1", "anular2", 
                "menique1", "menique2",
                "pulgar1", "pulgar2"]

nl_sep = ["pulgarindice",  "indicecorazon", 
            "corazonanular", "anularmenique"]

not_allowed = ["hand.wrist", "index.d_phalanx", "middle.d_phalanx", "ring.d_phalanx", "pinky.d_phalanx", "thumb.d_phalanx"]

num_sockets = 11
##### End Global Variables #####

if __name__ == '__main__':

    # Get the controller of the scene, so we can manipulate it
    cont = bge.logic.getCurrentController()
    hand = cont.owner
    
    # Block of code executed only once
    if not hand['StartUp']:
    	#logger_file = "udp_blenthread" + str(time.time()) + DATE + ".log"
        logging.basicConfig(filename='debugger.log',level=logging.DEBUG)
        #logging.basicConfig(filename='logger_file',level=logging.INFO) # Recomended
        GL.logging = logging
        GL.logging.info("StartUp setup")
        GL.logging.info("Only once block START: " + str(time.time()))

        

        # Array of bones extracted from the scene
        bones_list = [bone for bone in hand.channels if bone.name not in not_allowed]
        GL.num_bones = len(bones_list)

        xBones = []

        for i in range(0, len(bones_list)):
            if i%2 == 0:
                xBones.append(bones_list[i]) 

        GL.bones_list = bones_list
        GL.xBones = xBones

        # Sockets setup
        s = [socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) for i in range(num_sockets)] #<----------------- include sockets for X
        socket_address = [address + nl_fingers[i] for i in range(num_sockets-1)]
        socket_address.append(address + "expansion")

        for address in socket_address:
            # Make sure the socket does not already exist
            try:
                os.unlink(address) # Remove (delete) the file path
            except OSError:
                if os.path.exists(address):
                    GL.logging.warning("OSError: " + address)
                    raise

        for i in range(num_sockets):
            s[i].setblocking(0)
            s[i].bind(socket_address[i])

        GL.s=s
        hand['StartUp'] = True
        GL.logging.info("Only-once block FINISH: " + str(time.time()))
    
    else:

        GL.logging.info("Threads' creation START: " + str(time.time()))
        # Creation of the threads (ALL OK)
        moverZ_threads = [threading.Thread(name=GL.bones_list[i], target=moverZ, args=[GL.bones_list[i], nl_fingers[i], hand, GL.s[i]]) for i in range(GL.num_bones)]
        moverX_thread = threading.Thread(name="btwFingers", target=moverX, args=[GL.xBones, nl_sep, hand, GL.s[-1]])

        for z in moverZ_threads:
            z.start()

        for z in moverZ_threads:
            z.join()

        moverX_thread.start()
        moverX_thread.join()
        GL.logging.info("Threads' creation FINISH: " + str(time.time()))

