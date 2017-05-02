# "Desarrollo de un entorno gráfico de usuario para guante de datos 5DT" (Development of a GUI for 5DT data glove)

## Synopsis

This repository contains all the code used in my End-Of-Degree project at the University of Valladolid. The directory structure is the exact one of the ROS packet in which it should be placed to run. 

## Motivation

The aim was creating the comunications layer between a "5DT Data Glove 14 Utra (Left)" and a VR enviroment (in this case Blender) implementing a publisher/subscriber paradigm in ROS, and developing an initial VR animation of the hand.

## Installation

All programs used within this project are published under a free license.

### Prerequisites

* [Ubuntu 14.04](https://www.ubuntu.com/) - The OS used (the ROS version depends on it)
* [ROS Índigo](http://wiki.ros.org/indigo) - The ROS version used was Indigo.
* [Blender 2.78](https://www.blender.org/) - Latest version of Blender at the time, using Python 3.5

### Install

I strongly recomend following the installation processes shown in the ros wiki. After creating a packet with the name "tfg", all we have to do is replacing the files created with the ones provided here

## Build and Run!

The software runs "out of the box". All we have to do is dive into the directory folder containing the GUI and run it:

```
$ cd ~/path_to_catkin_ws/catkin_ws/src/tfg/scripts && python3 gui_script.py
```

and launch them in the following order:

```
Compile -> Launch ROSCore -> Launch Glove -> Launch Publishers -> Launch Blender -> Launch Subscriptors
```



## API Reference

The full project is in the docs folder. Annex I contains a user guide.

## License

All this code is published under [CC-BY-SA](https://creativecommons.org/licenses/by-sa/4.0/legalcode)
