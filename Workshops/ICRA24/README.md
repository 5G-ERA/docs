## Connected Robotics Platform for ROS Deployment in Real-world Network Settings – ICRA Tutorial Proposal

## Summary:
Internet is a dynamic and unstructured network with multi-domain and multi-administration. ROS and ROS2(DDS) are not designed for the unstructured network, hence won’t work in real-world deployment. We need an effective mechanism to unblock ROS traffic and orchestrate internet resources for autonomous robots. The mechanism could accelerate the transformation from existing cloud robotics into the future of 5G/6G enabled connected robotics. This tutorial presents a ROS extension called Connected Robotics Platform (CROP) to extend the ROS from only working in structured lab environments into fully scalable at real-world CONNECT+ settings. CROP is an open source, cloud native implementation of Network Function Virtualisation (NFV) which optimised specifically for robot development. Existing ROS applications enhanced by CROP can be deployed natively into large-scale distributed systems without modification, at the same time maintain the necessary performance and security. Furthermore, a radioaware semantic mapping framework is introduced. By combining radio signal quality information with semantic mapping techniques, the framework creates a comprehensive understanding of the network environment, which enhances the decision-making of the autonomous robots. The development is a step forward to 5G/6G  enabled connected robotics on supporting robot deployment in real-world settings at the same time optimising behaviours of robots according to digital infrastructures.


## Previous events:

* Hackathon: 5G Enhanced Robot Autonomy hackathon, April 2023, organised by 5G-ERA
consortium. \
URL: https://5g-ppp.eu/event/5g-era-hackathon/. 
* Workshop: 6G enabled Network Applications for the Future of Connected Robotics, June 2023, organised by 5G-ERA consortium and one6G Association. \
URL: https://www.eucnc.eu/programme/special-sessions/special-session-9/. 


## Preliminary Agenda

* [Preliminary Agenda ClickMe!](#Preliminary_Agenda)


## Demonstration Videos:

* Connected Robotics Platform for ROS in an \
Unstructured Network Environment

[<img src="https://img.youtube.com/vi/KtKC98YrqK0/maxresdefault.jpg" width="50%">](https://youtu.be/KtKC98YrqK0)

* Towards Smarter 5G-Connected Mobile Robots: Dynamic \
Offloading and Radio-Aware Semantic Maps

[<img src="https://img.youtube.com/vi/CMcDZyFyge8/maxresdefault.jpg" width="50%">](https://youtu.be/CMcDZyFyge8)

* Collision avoidance demonstration

[<img src="https://img.youtube.com/vi/4RbB8IuZZy8/maxresdefault.jpg" width="50%">](https://youtu.be/4RbB8IuZZy8)

* Train detector demonstration

[<img src="https://img.youtube.com/vi/xiZsWGed9FU/maxresdefault.jpg" width="50%">](https://youtu.be/xiZsWGed9FU&t)

* Transport Robot

[<img src="https://github.com/5G-ERA/docs/blob/main/Workshops/ICRA24/files/Transport.png" width="50%">](https://drive.google.com/file/d/1nE6pK1BZHgOEQWO3PwDbNSTkKrUXQwMh/view?usp=sharing)

* Surveillance Robot

[<img src="https://github.com/5G-ERA/docs/blob/main/Workshops/ICRA24/files/Survilience.png" width="50%">](https://drive.google.com/file/d/1Gv4h6ThK_YSgq8uCPa-nv6ZkcuSTKtZh/view?usp=sharing)

* Manufacturing Robot

[<img src="https://github.com/5G-ERA/docs/blob/main/Workshops/ICRA24/files/Manufacturing.png" width="50%">](https://drive.google.com/file/d/1PngJkUEGcL0MKmtqTNpuv9ED02qb5Y24/view?usp=sharing)


* 5G Testbed

[<img src="https://github.com/5G-ERA/docs/blob/main/Workshops/ICRA24/files/Testbed.png" width="50%">](https://drive.google.com/file/d/1s5CeFMR5aM87kWybYq3Paf0xL4JKwE-r/view?usp=sharing)

<div id="Preliminary_Agenda"></div>

## Preliminary Agenda:
**Introduction and welcome** (10 mins) - _Prof Renxi Qiu (University of Bedfordshire, UK)_

**Section A:** Connected Robotics Platform for ROS in an Unstructured Network Environment (1h 40 mins) By _Dr. Michal Kapinus (Brno University of Technology, Czechia), Mr. Bartosz Bratus (University of Bedfordshire, UK) and Mr. Matúš Kašuba (BringAuto, Czechia)_

**Presentation:**
* Connected Robotics Platform - System architecture and communication framework (20 mins)

**Tutorial:**
* Creating a standard ROS application for object detection with camera (20 mins)
* Go through the CROP communication examples with the hosts, configuring security and relay to adapt the network topology of the internet (20 mins)
* Deploy the newly created standard ROS application to a remote infrastructure e.g., AWS (20 mins)
* Consuming the newly created ROS application in a standard ROS environment (20 mins)

**Break (10 mins)**

**Section B:** Robots adapt themself according to network environments (1h 40 mins) By _Dr. Lanfranco Zanzi (NECLab, Germany), Mr. Adrian Lendínez (Telefonica, Spain) and Mr. Guillem Garí (RobotNik Automation SLL, Spain)_

**Presentation:**
* Connected Robotics Platform Dynamic Offloading and Radio-Aware Semantic Maps (20
mins)

**Tutorial:**
* Creating a Radio-Aware Semantic Maps for a given simulation environment (30 mins)
* Robot optimises its navigation trajectory according to the 5G quality signal (20 mins)
* Edge and slice switch over according to the condition of networks (30 mins)
