#      cal_sub.py
# Developed by Pablo San Jose                 
#     License CC-BY-SA    ***/

from listenerGeneric import *

#########            Main            #########

if __name__ == '__main__':
    
    setupLoggerFile('cal_sub', 'ERROR')

    # Spawning of the processes
    c=[]	
    c.append(Process(target=launchListenerCalibration, args=("calibration_max", 20)))
    c.append(Process(target=launchListenerCalibration, args=("calibration_min", 21)))

    # Launching
    for cp in c:
        cp.start()

    # Joining
    for cp in c:
        cp.join()

#########            Main            #########