#      gui_script.py
# Developed by Pablo San José                 
#     License CC-BY-SA    ***/

#!/usr/bin/python3

import tkinter as tk
import tkinter.messagebox
from PIL import ImageTk, Image
from subprocess import call as command
import os
import signal
from multiprocessing import Process

#####  Constants  #####

logging_levels=["DEBUG", "INFO", "WARNING"]
logging_value=2

##### End Constants #####

#####  Callbacks  #####

def gloveCallBack():
    tk.messagebox.showinfo( "Atention!", "Launching\nGlove Driver")
    launcher = Process(target=gloveLauncher, args=())
    launcher.start()

def manoCallBack():
    tk.messagebox.showinfo( "Atention!", "Launching\nROS Publishers")
    launcher = Process(target=manoLauncher, args=())
    launcher.start()

def subscriptorsCallBack():
    tk.messagebox.showinfo( "Atention!", "Launching\nROS Listeners")
    launcher = Process(target=subsLauncher, args=())
    launcher.start()

def calibrationCallBack():
    tk.messagebox.showinfo( "Atention!", "Launching\nROS Listeners")
    launcher = Process(target=calSubsLauncher, args=())
    launcher.start()
    
def blenderCallBack():
    tk.messagebox.showinfo( "Atention!", "Launching\nBlender")
    launcher = Process(target=blenderLauncher, args=())
    launcher.start()

def roscoreCallBack():
    tk.messagebox.showinfo( "Atention!", "Launching\nROSCore")
    launcher = Process(target=roscoreLauncher, args=())
    launcher.start()

def compilationCallBack():
    tk.messagebox.showinfo( "Compilation", "Compilation\nunder way")
    launcher = Process(target=compilationLauncher, args=())
    launcher.start()
    
##### End Callbacks #####

##### Launchers #####

def gloveLauncher():
    command(["xterm", "-e", "sudo" ,"../../../devel/lib/tfg/dglove"])

def manoLauncher():
    command(["xterm", "-e", "../../../devel/lib/tfg/mano"])

def calSubsLauncher():
    command(["xterm", "-e", "python", "./cal_sub.py"])

def subsLauncher():
    command(["xterm", "-e", "python", "./main_sub.py"])

def blenderLauncher():
    command(["xterm", "-e", "blender", "./dgHand.blend"])

def roscoreLauncher():
    command(["xterm", "-e", "roscore"])

def compilationLauncher():
    command(["xterm", "-e", "./compilation.sh"])

##### End Launchers #####

def finish():
    os.killpg(os.getpid(), signal.SIGTERM)

root = tkinter.Tk()

root.wm_title("Interfaz Gráfico para 5DT Data Glove")

# R0 C0-4
title_image = ImageTk.PhotoImage(Image.open("./img/titulo2.png"))
title = tk.Label(root, image=title_image)
title.grid(row=0, columnspan=5, sticky='W'+'E'+'N'+'S', padx=5, pady=5)

# R1-2 C0
glove_launcher = tk.Button(text="Launch\nGlove", command=gloveCallBack)
glove_launcher.grid(row=1, rowspan=2, column=0, sticky='W'+'E'+'N'+'S', padx=5, pady=5)

# R1-2 C1
mano_launcher = tk.Button(text="Launch\nPublishers", command=manoCallBack)
mano_launcher.grid(row=1, rowspan=2, column=1, sticky='W'+'E'+'N'+'S', padx=5, pady=5)

# R1-2 C2
ros_icon = ImageTk.PhotoImage(Image.open("./img/ros_icon.png"))
ros_label = tk.Label(root, image=ros_icon)
ros_label.grid(row=1, rowspan=2, column=2, sticky='W'+'E'+'N'+'S', padx=5, pady=5)

# R1 C3
subscriptors_launcher = tk.Button(text="Launch\nSubscriptors", command=subscriptorsCallBack)
subscriptors_launcher.grid(row=1, rowspan=1, column=3, sticky='W'+'E'+'N'+'S', padx=5, pady=5)

# R2 C3
calibration_launcher = tk.Button(text="Launch\nCal Subs", command=calibrationCallBack)
calibration_launcher.grid(row=2, rowspan=1, column=3, sticky='W'+'E'+'N'+'S', padx=5, pady=5)

# R1 C4
blender_launcher = tk.Button(text="Launch\nBlender", command=blenderCallBack)
blender_launcher.grid(row=1, rowspan=2, column=4, sticky='W'+'E'+'N'+'S', padx=5, pady=5)

# R3 C2
roscore_launcher = tk.Button(text="Launch\nROSCore", command=roscoreCallBack)
roscore_launcher.grid(row=3, rowspan=1, column=2, sticky='N', padx=5, pady=0)

# R3 C0
compilation_launcher = tk.Button(text="Compile", command=compilationCallBack)
compilation_launcher.grid(row=3, rowspan=1, columnspan=2, column=0, sticky='N' + 'S' + 'E' + 'W', padx=5, pady=0)

# LAST
quit = tk.Button(text="QUIT", fg="red", command=finish)
quit.grid(row=8, column=4, padx=1, pady=1)

root.mainloop()

#####  End script  #####