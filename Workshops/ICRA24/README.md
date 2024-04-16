## Connected Robotics Platform for ROS Deployment in Real-world Network Settings – ICRA Tutorial

[ICRA24, Yokohama, Japan 13th May 2024 from 13:00-17:30PM (JST GMT+9)](https://2024.ieee-icra.org/program/#Workshops-Tutorials)

## Summary:
Internet is a dynamic and unstructured network with multi-domain and multi-administration. ROS and ROS2(DDS) are not designed for the unstructured network, hence won’t work in real-world deployment. We need an effective mechanism to unblock ROS traffic and orchestrate internet resources for autonomous robots. The mechanism could accelerate the transformation from existing cloud robotics into the future of 5G/6G enabled connected robotics. This tutorial presents a ROS extension called [Connected Robotics Platform (CROP)](https://github.com/5G-ERA/docs) to extend the ROS from only working in structured lab environments into fully scalable at real-world CONNECT+ settings. CROP is an open source, cloud native implementation of Network Function Virtualisation (NFV) which optimised specifically for robot development. Existing ROS applications enhanced by CROP can be deployed natively into large-scale distributed systems without modification, at the same time maintain the necessary performance and security. Furthermore, a radioaware semantic mapping framework is introduced. By combining radio signal quality information with semantic mapping techniques, the framework creates a comprehensive understanding of the network environment, which enhances the decision-making of the autonomous robots. The development is a step forward to 5G/6G  enabled connected robotics on supporting robot deployment in real-world settings at the same time optimising behaviours of robots according to digital infrastructures.


<div id="Agenda"></div>

## Agenda:
[<img src="https://github.com/5G-ERA/docs/blob/main/Workshops/ICRA24/files/Agenda.jpg"  width="800" >](https://youtu.be/KtKC98YrqK0)


**13:00-13:15 Registration and environment setup**

**13:15-13:30 Opening and introduction**

Connected Robotics Platform (CROP) – Network softwarization and virtualization for robotics - _Prof Renxi Qiu (University of Bedfordshire, UK)_

**13:30-14:50 Demonstration of the Connected Robotics**

Scenario: Network-based SLAM in unstructured real-world network using CROP
* Offloading SLAM to from robots to Edge – _Dr Michal Kapinus (Brno University of Technology, Czechia)_
* Quality-aware robot operations, robot selecting optimal resource based on quality signals – _Dr Lanfranco Zanzi (NECLab, Germany)_
* Resilient and robust service by autonomous Edge switch over – _Mr. Bartosz Bratus (University of Bedfordshire, UK)_

Integrated demonstration (hybrid): 
*	Remote demonstration: Summit XL connected to AWS Cloud for PPDR surveillance tasks – _Mr. Guillem Garí and Ms. Sandra Moreno Olivares (RobotNik Automation SLL, Spain), Mr. Adrian Lendínez (Telefonica, Spain) and Mr. Vladimir Guroma (University of Bedfordshire, UK)_

Presentations:
*	Applications of Connected Robotics in Transportation – _Mr. Jan Kubálek (BringAuto, Czechia)_
*	Applications of Connected Robotics in Manufacturing – _Mr. Kandarp Amin and Prof Darren Williams (TWI, UK), Mr. Sebastian Andraos (HAL-Robotics, UK)_

**14:50-15:40 Hand on experience of ROS under the Connected Robotics**

Scenario: Breaking the barrier of ROS2 DDS into real-world networks
* Limitation of ROS2/DDS and Hand on experience of remote object detection in unstructured networks – _Dr Michal Kapinus (Brno University of Technology, Czechia)_
* Hand on experience of obstacle avoidance in unstructured networks – _Dr Michal Kapinus (Brno University of Technology, Czechia)_
* Demonstration: Behind scene story, fully orchestrated infrastructure automation – _Mr. Bartosz Bratus and Mr. Radu Popescu (University of Bedfordshire, UK)_

**15:40-16:00 Break**

**16:00-17:20 Building your own ROS applications for the Connected Robotics**

Scenario: Accelerated ROS application development
*	Demonstration: Relay in connected robotics – _Dr Michal Kapinus (Brno University of Technology, Czechia)_
*	Demonstration: How to build a tele-operation app in unstructured network – _Mr. Angelos Stathis (WINGS ICT Solution, Greece)_
*	Demonstration: How to build a containersied portable region detector for industrial Robots – Mr. Paul McHard &  Mr. Sebastian Andraos (HAL-Robotics, UK)_
*	Demonstration: Onboarding and deploying the newly created network applications with orchestration – _Mr Bartosz Bratus and Mr. Radu Popescu (University of Bedfordshire, UK)_
*	Hand on experience to the newly created ROS applications for connected robotics _Users, assisted by all_

**17:20-17:30 Closing remarks**



## Demonstration Videos:

CROP for ROS in an Unstructured Network Environment | Radio-Aware Semantic Maps
:-: | :-:
[<img src="https://img.youtube.com/vi/KtKC98YrqK0/maxresdefault.jpg"  width="400" height="265">](https://youtu.be/KtKC98YrqK0) | [<img src="https://img.youtube.com/vi/CMcDZyFyge8/maxresdefault.jpg" width="400" height="265">](https://youtu.be/CMcDZyFyge8&t)

Collision avoidance demonstration | Train detector demonstration
:-: | :-:
[<img src="https://img.youtube.com/vi/4RbB8IuZZy8/maxresdefault.jpg"  width="400" height="265">](https://youtu.be/4RbB8IuZZy8) | [<img src="https://img.youtube.com/vi/xiZsWGed9FU/maxresdefault.jpg"  width="400" height="265">](https://youtu.be/xiZsWGed9FU&t)


Transport Robot | Surveillance Robot
:-: | :-:
[<img src="https://img.youtube.com/vi/J19jpORf4Po/maxresdefault.jpg"  width="400" height="265">](https://www.youtube.com/watch?v=J19jpORf4Po) | [<img src="https://img.youtube.com/vi/-apRZ1EWGo0/maxresdefault.jpg"  width="400" height="265">](https://www.youtube.com/watch?v=-apRZ1EWGo0) 



Manufacturing Robot | 5G Testbed
:-: | :-:
[<img src="https://img.youtube.com/vi/t5Q4KgpOAp0/maxresdefault.jpg"  width="400" height="265">](https://www.youtube.com/watch?v=t5Q4KgpOAp0) | [<img src="https://img.youtube.com/vi/MQnypIggWu0/maxresdefault.jpg"  width="400" height="265">](https://www.youtube.com/watch?v=MQnypIggWu0) 



## Previous events:

* Hackathon: 5G Enhanced Robot Autonomy hackathon, April 2023, organised by 5G-ERA
consortium. \
URL: https://5g-ppp.eu/event/5g-era-hackathon/. 
* Workshop: 6G enabled Network Applications for the Future of Connected Robotics, June 2023, organised by 5G-ERA consortium and one6G Association. \
URL: https://www.eucnc.eu/programme/special-sessions/special-session-9/. 

