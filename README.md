# Connected Robotics Platform (CROP) - Documentation

CROP enables seamless interaction among robots and Cloud/Edge devices. With the new capability, distributed robots’ development with Robot Operating System (ROS) is significantly simplified. CROP is highly efficient, secure, fully open source and vendor independent. The topology of the infrastructure is completely transparent to robots under the CROP. Therefore, there is little difference to robot developers that the device is running remotely via the internet on cloud or locally in local area network (LAN). 


## Contents

This repository contains the technical information about different components of the 5G-ERA and proposals for workshops organized by the 5G-ERA consortium.

* [Middleware](./Middleware/readme.md) - Middleware for the on demand deployment of the vertical applications
* [NetApp](./NetApp/readme.md) - Network Application framework description and description of the in-house Network Applications developed by the 5G-ERA consortium
* [Workshops](https://github.com/5G-ERA/docs/tree/main/Workshops/Athens23) - list of the workshops prepared by the 5G-ERA consortium.
* Resources - folder containing additional resources that help get familiar with the 5G-ERA project.




| Technical Challenge  | Potential Users          |  Scenarios |
|----------|:-------------:|------:|
| ROS application to be running in the cloud and interact with robot |  Robot Application Developers | Tutorial|
| Robot interaction within unstructured network |    Robot Application Developers  |  Tutorial  |
| Role based access control | System Administrators |  Tutorial   |
| Netapp on multiple edges and locations | Network Application Developers |  Tutorial   |
    




  
         
# Pre-required step

Please go through the prerequisites and [middleware installation tutorial](https://github.com/5G-ERA/docs/tree/main/Middleware/architecture/Middleware%20Installation%20guide)

## Possible Scenarios, technical challenges and solutions by CROP

# Scenario 1: 

## Potential Users : 
Robot Application Developers 

### Example : 

```
I want my ROS application to be deployed in the cloud and Interact with my local robots. 

Problem: Although my ROS code works fine in LAN, it does not work anymore through the Internet.
```
### Technical Challenge : 
The Internet has multiple domains and multi administrations. Various security requirements and package inspection kill my existing ROS application. 

### CROP Solution : 
CROP integrates network function virtualisation into robot deployment. 
It generates a static virtual LAN for ROS applications, at the same time dynamically adapting the network resources for the virtual network on demand.  
Domain knowledge of Robotics and ICT are encapsulated separately.

### Tutorial/Documentation : 

Step 1: Onboard your custom robot from the [onboarding robot tutorial](https://github.com/5G-ERA/docs/blob/main/Middleware/architecture/Onboarding/New%20robot%20onboarding.md)

Step 2: Onboard the ROS network application from the [tutorial](https://github.com/5G-ERA/docs/blob/main/Middleware/architecture/Onboarding/Netapp%20Onboarding.md) 

Step 3: Trigger the Deployment of  your ROS network application


# Scenario 2:

## Potential Users: 
Robot Application Developers

### Example:
```
I want to give the best possible  resources to my mobile robots. Although, there the answer is not fixed. It could be either 5G or WiFi; either Local Edge or Remote Cloud; either eMBB or URLLC slice.

Problem: The task is cumbersome and completely out of my knowledge. At the end, I just use VPN and hard code everything to the robot. 
```
### Technical Challenge: 
•	Network topology for mobile robots is dynamic. 

•	Knowledge gap between robotics and ICT.

### CROP Solution:

Please follow the documentation.
### Tutorial/Documentation:
Step 1: Register and onboard your robot 

Step 2: Tutorial will be added (Configuring of the infrastructure of the testbed) slice manager

Step 3: Configuration of new or existing tasks for utilising [slice mechanism](https://github.com/5G-ERA/middleware/blob/0d21429019390f2f96e29266185a1cb75eade1a0/docs/Developer/5g_testbed_integration.md) 

Step 4: Trigger the Deployment of  your ROS network application

# Scenario 3:

## Potential Users 
System administrators 

### Example:
```
I want a secured communication between robot and ROS application offloaded in the cloud. 

Problem: I have no idea what is running in the robot. I might break the system completely.
```

### Technical Challenge:
Scalability and maintainability of robot applications.

### CROP Solution:
CROP utilises cloud native design to separate vertical logic from horizontal deployment. It enables encrypted end to end communication and provides role-based access at the container networking level which is fully encapsulated from ROS applications. Furthermore, CROP supports infrastructure as code (IaC) to automate the management process of the system administrators.

### Tutorial/Documentation:
Step 1(optional): Additional configuration of the middleware  (https, domain name)

Step 2(optional): Identity Management tutorial, role based control (RBAC)  

# Scenario 4:

## Potential Users: 
Network Applications Developers

### Example:
```
I want my network application to be roaming on multiple edges (available in multiple locations), while maintaining real-time ROS communication to the robots.

Problem: The development is time consuming and can hardly be reused. 
```

### Technical Challenge:
•	Synchronisation of data.

•	Maintain stateless and stateful transactions in operation. 

•	Integrating robot specific domain knowledge into the resource provision

### CROP Solution:
CROP enables Platform as a Service for dynamic resource provision. It orchestrates robot specific operations together with network resources for the best quality of experience. 
CROP supports event sourcing and CQRS for schema on read. It enables knowledge to be better reused in unexpected situations.  

### Tutorial/Documentation:
Step 1: Go through the [Relay network application tutorial](https://github.com/5G-ERA/relay_network_application) 

Step 2: Onboard the [ROS network application](https://github.com/5G-ERA/docs/blob/main/Middleware/architecture/Onboarding/Netapp%20Onboarding.md) from the tutorial 

Step 3: Trigger the Deployment of  your ROS network application















