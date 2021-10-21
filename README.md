# ViveServer

Python package for using HTC Vive trackers for motion capture. Originally developed for 1/10th scale RC cars.

Features a central server with a user interface for monitoring devices and calibrating VR frame of reference. 
Broadcasts tracker data over UDP multicast, which can be subscribed to by clients and forwarded over a ROS2 interface. 


Known Issues:
1. Latency can increase when used over WiFi with a router with IGMP snooping disabled. For best results, turn IGMP snooping on.
