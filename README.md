# Connected Robotics Platform (CROP) - Documentation

This repository contains the technical documentation of CROP, the 5G-ERA project.

This documentation contains the information about the 5G-ERA project components. Mainly Middleware, Network Applications and Workshops organized by the consortium. 

Middleware and Network Applications explain what are these components and how they allow efficient cooperation between robots and vertical applications. 

## Contents

This repository contains the technical information about different components of the 5G-ERA and proposals for workshops organized by the 5G-ERA consortium.

* [Middleware](./Middleware/readme.md) - Middleware for the on demand deployment of the vertical applications
* [NetApp](./NetApp/readme.md) - Network Application framework description and description of the in-house Network Applications developed by the 5G-ERA consortium
* [Workshops](./Workshops/readme.md) - list of the workshops prepared by the 5G-ERa consortium.
* [Resources](./Resources/readme.md) - folder containing additional resources that help get familiar with teh 5G-ERA project.


## System Architecture

Explanation of the architecture from the very high-level POV.

## What is CROP? 

CROP enables seamless interaction among robots and Cloud/Edge devices. With the new capability, distributed robots’ development with Robot Operating System (ROS) is significantly simplified. CROP is highly efficient, secure, fully open source and vendor independent. The topology of the infrastructure is completely transparent to robots under the CROP. Therefore, there is little difference to robot developers that the device is running remotely via the internet on cloud or locally in local area network (LAN). 

### Possible Scenarios, technical challenges and solutions by CROP

## Scenario 1: 
* Potential Users : Robot Application Developers 

```
Example : I want my ROS application to be deployed in the cloud and Interact with my local robots. 
Problem, although my ROS code works fine in LAN, it does not work anymore through the Internet.

Technical Challenge : The Internet has multiple domains and multi administrations. Various security requirements and package inspection kill my existing ROS application. 

CROP Solution : CROP integrates network function virtualisation into robot deployment. 
It generates a static virtual LAN for ROS applications, at the same time dynamically adapting the network resources for the virtual network on demand.  
Domain knowledge of Robotics and ICT are encapsulated separately.

Tutorial/Documentation : (Pre-required Step1: that is needed to be followed in every scenario mentioned*) Go through the prerequisites and middleware installation tutorial
Step 2: Onboard your custom robot from the onboarding robot tutorial 
Step 3: Onboard the ROS network application from the tutorial 
Step 4: Trigger the Deployment of  your ROS network application
```

## Scenario 2:

* Potential Users: Robot Application Developers

```




