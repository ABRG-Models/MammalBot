# MammalBot

Computational models and related work for the MammalBot cognitive architecture.

*This open source software code was developed in part or in whole in the Human Brain Project, funded from the European Union’s Horizon 2020 Framework Programme for Research and Innovation under the Specific Grant Agreement No. 785907 (Human Brain Project SGA2).*

## Overview

MammalBot is a layered control system architecture modelled on the mammalian brain capable of generating motivated real-time behaviour on a range of different target physical robot platforms. The system is composed of a set of nested sensorimotor loops in which lower loops can function without the help of higher loops, whilst higher loops operate by modulating the behaviour of those lower down.

## Documentation

Information on how to install the software required for the MammalBot architecture and how to develop or use the models may be found in [the wiki](https://github.com/ABRG-Models/MammalBot/wiki)

## Technical integration

To avoid generating onerous constraints on flexible development, MammalBot uses an agile approach based around a preferred set of software tools. This framework, illustrated below, is intended to maximise interoperability between different kinds of models and to minimise the difficulty of deployment on robot hardware. Solid lines indicate native compatibility between
components, and dashed lines indicate where compatibility may be achieved via a conversion or wrapper utility.

![alt text](./images/integration.png "MammalBot Infrastructure")

# Motivational system

The motivational system of the Mammalbot architecture has been implemented using two approaches:

[A dynamical systems one](models/python/hypothalamus)
[A neuro-computational one](models/matlab)

Documentation about how to run it can be found following each of the links.

# Further reading

## BRAHMS

* [General documentation](http://brahms.sourceforge.net/docs/)

* [BRAHMS Manager](http://brahms.sourceforge.net/docs/BRAHMS%20Manager.html)

* [Python process development tutorial](http://brahms.sourceforge.net/docs/Quick%20Start%20(1262).html)

* [Python component bindings reference](http://brahms.sourceforge.net/docs/Python%20(1262).html)
  
  ## MiRo

* [General documentation](http://labs.consequentialrobotics.com/miro-e/docs/)

* [ROS interfaces](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS_interface)
  
  ## SpineML / SpineCreator

* [General documentations](http://spineml.github.io)

* [SpineCreator tutorial](http://spineml.github.io/spinecreator/tutorial/)
