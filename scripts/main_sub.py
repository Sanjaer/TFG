#      main_sub.py
# Developed by Pablo San Jose                 
#     License CC-BY-SA    ***/

#!/usr/bin/python

from listenerGeneric import *

#########            Main            #########

if __name__ == '__main__':

    setupLoggerFile('main_sub', 'INFO', 'w')

    logging.info("Processes creation START" + str(time.time()))

    p=[]
    # Code to launch processes together
    p = [Process(target=launchListenerFingers, args=(nl_fingers[i], i)) for i in range(len(nl_fingers))]
    p.append(Process(target=launchListenerExp, args=("sep", 11))) # Proccess for interdigital sensors

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

#########            Main            #########