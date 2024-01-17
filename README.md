# Connected Robotics Platform (CROP) - Documentation

[![alt text](<assets/icons without padding/twitter.png>)][1] [![alt text](<assets/icons without padding/facebook.png>)][1] 
[<img src="image.png" width="20px" />](https://www.youtube.com/channel/UCFn5FI9OYLA9_jTwl2cwdFA/featured) 
[<img src="image-1.png" width="20px" />](https://www.linkedin.com/company/5g-era-project/%20)


CROP enables seamless interaction among robots and Cloud/Edge devices. With the new capability, distributed robots’ development with Robot Operating System (ROS) is significantly simplified. CROP is highly efficient, secure, fully open source and vendor independent. The topology of the infrastructure is completely transparent to robots under the CROP. Therefore, there is little difference to robot developers that the device is running remotely via the internet on cloud or locally in local area network (LAN). 

*CROP was developed under the funding of the European Union's Horizon 2020 Research and Innovation programme under grant agreement No. 101016681.*

# Index:

- [1. Introduction](#1-introduction)
    - [1.1 CROP target audience](#11-crop-target-audience)
    - [1.2 Why to use CROP - use cases.](#12-why-to-use-crop---use-cases)
    - [1.3 Authors and user support.](#13-authors-and-user-support)
    - [1.4 Limitations - ROS Only support.](#14-limitations)
    - [1.5 License.](#15-licence)

- [2. Installation](#2-installation)
- [3. Getting started.](#3-getting-started)
    - [3.1 CROP components.](#31-crop-components)
        - [3.1.1 Middleware.](#311-middleware)
        - [3.1.2 Network applications & relay.](#312-network-applications--relay)
        - [3.1.3 Dashboard.](#313-dashboard)
    - [3.2 User roles.](#32-user-roles)
        - [3.2.1 Administrator](#321-administrator)
        - [3.2.2 CROP developer](#322-crop-developer)
        - [3.2.3 Robot developer](#323-robot-developer)
        - [3.2.4 Network application Developer](#324-network-application-developer)
    - [3.3 System provision & onboarding](#33-system-provision--onboarding)
        - [3.3.1 Deployment of middlewares & centralAPI.](#331-deployment-of-middlewares--centralapi)
        - [3.3.2 Accounts & Profiling - user onboarding.](#332-accounts--profiling---user-onboarding)
        - [3.3.3 Robot fleet onboarding.](#333-robot-fleet-onboarding)
        - [3.3.4 Network app onboarding & task definition.](#334-network-app-onboarding--task-definition)
    - [3.4 Task deployment into the robot.](#34-task-deployment-into-the-robot)
        - [3.4.1 Connect robot to local middleware.](#341-connect-robot-to-local-middleware)
        - [3.4.2 Task deployment & network app consumption](#342-task-deployment--network-app-consumption)         
- [4. Advanced topics.](#3-advanced-topics)
    - [4.1 5G-Testbed integration & slice management.](#41-5g-testbed-integration--slice-management)
    - [4.2 Middleware policies.](#42-middleware-policies)
- [5. Useful learning resources.](#5-useful-learning-resources)
    - [5.1 Youtube channel and tutorials.](#51-youtube-channel-and-tutorials)
    - [5.2 Online tutorials](#52-online-tutorials)
    - [5.3 ICRA 2024 workshops.](#53-icra-2024-workshops)
- [6. Social Media and contact information.](#6-social-media-and-contact-information)

    
# 1. Introduction:

The aim of the CROP package is the seamless integration and easy to use solution for ROS robots to utilice and offload on-demand to the digital enviroment i.e Edge or Clouds, in order to complete their given tasks faster, eficiently and with greater computational capabilities using cloud-computer and containerisation & orchestration technologies. In addition, CROP supports various 5G slicing technologies. CROP provides a vendor-free solution for ROS to extend its connectivity in a various of ways that solve some of the main problems like single network domain and administration access to the network. 

## 1.1 CROP Target audience:

CROP is developed as a solution to the ROS-Developer and provides and abstraction layer that allows for easy hands-on experience with cloud-computing addressing the main limitations of ROS connecivity and security issues with regards to the networking implementation.

## 1.2 Why to use CROP - Use cases:

Significant effort has been done by the new ROS 2 to solve some of the problems of its predecesor and introducing DDS as the main connectivity mechanism. However, ROS 2 still limited to structured/industry enviroments where:

    - QoS is fixed.
    - Networkm topology is static.
    - Single domain.
    - Single administration.

This, however, limits ROS robots to approach the more realistic use cases and use cloud-native design further done VPN solutions. Reality imposes the following rules:

    - QoS changes constantly.
    - Network topology is dynamic.
    - Multi-domains co-exits.
    - Multi administration with different security requirements.

Following are some use-cases where CROP comes in handy.

<details>

<summary>List of use-cases - Click to expand. </summary>

**Example 1:** 
```
I want my ROS application to be deployed in the cloud and Interact with my local robots. 

Problem: Although my ROS code works fine in LAN, it does not work anymore through the Internet.

🔳 Technical Challenge :

The Internet has multiple domains and multi administrations. Various security requirements and package inspection kills my existing ROS application.

🔳 CROP Solution :

CROP integrates network function virtualisation into robot deployment. It generates a static virtual LAN for ROS applications, at the same time dynamically adapting the network resources for the virtual network on demand.
Domain knowledge of Robotics and ICT are encapsulated separately.
```

**Example 2:**
```
I want to give the best possible resources to my mobile robots. Although, there the answer is not fixed. It could be either 5G or WiFi; either Local Edge or Remote Cloud; either eMBB or URLLC slice.

Problem: The task is cumbersome and completely out of my knowledge. At the end, I just use VPN and hard code everything to the robot. 

🔳 Technical Challenge:
Network topology for mobile robots is dynamic. Also there exits a big knowledge gap between robotics and ICT.

🔳 CROP Solution:
The CROP Middleware allows for automatic resource planning optimization based on ML, tracking of hearbeat and system administration policies.
```

**Example 3:**
```
I want a secured communication between robot and ROS application offloaded in the cloud. 

Problem: I have no idea what is running in the robot. I might break the system completely.

🔳 Technical Challenge:
Scalability and maintainability of robot applications.

🔳 CROP Solution:
CROP utilises cloud native design to separate vertical logic from horizontal deployment. It enables encrypted end to end communication and provides role-based access at the container networking level which is fully encapsulated from ROS applications. Furthermore, CROP supports infrastructure as code (IaC) to automate the management process of the system administrators

```
**Example 4:**

```
I want my network application to be roaming on multiple edges (available in multiple locations), while maintaining real-time ROS communication to the robots.

🔳 Problem: The development is time consuming and can hardly be reused. 
Technical Challenge:
• Synchronisation of data.

• Maintain stateless and stateful transactions in operation.

• Integrating robot specific domain knowledge into the resource provision

🔳 CROP Solution:
CROP enables Platform as a Service for dynamic resource provision. It orchestrates robot specific operations together with network resources for the best quality of experience. CROP supports event sourcing and CQRS for schema on read. It enables knowledge to be better reused in unexpected situations.
```


</details>


## 1.3 Authors and user support:

The CROP package was developed by the 5G-ERA project [consortium](https://5g-era.eu/consortium/) with funding of the European Union's Horizon 2020 Research and Innovation programme under grant agreement No. 101016681.

Visit the 5G ERA project [website](https://5g-era.eu) for more information and for [contact](https://5g-era.eu/contact/).

## 1.4 Limitations:

Following are listed some of the limitations identified in the product:

🔴 Network applitcations have different hardware resources, therefore edge and cloud requriments for the CROP may change and it needs to be provided and deployed.

🔴 CROP is limited to ROS framework for robotics. However, network applications are ROS agnostic. For more information visit [network-apps]().

🔴 Middleware does not work outside of kubernetes cluster (both advantage and limitation).

🔴 The package of Middleware, NetApp Client, Relay Server supports only ROS2, altough other application can still be deployed as cloud native applications.

🔴 Currently , the first hand cloud support is only available for AWS.

🔴 Containerization of existing applications is necessary to manage them using Middleware.

## 1.5 Licence

For the licensing information see [LICENSE](LICENSE).

# 2. Installation

There are two types of installation: cloud base and local base. Both use terraform scripts.

TODO: terraform examples for cloud installation with AWS.

TODO: terraform local machine --> k8 credentials

The CROP solution is a distributed system with a central-api in your infrastructure topology. You will need to install the middleware in each of the machines of your distributed system.

The CROP solution is a distributed system with a central-api in your infrastructure topology. You will need to install the middleware in each of the machines of your distributed system.

```
sudo apt-get install ros-$(rosversion -d)-crop
```
Additionally, one central api needs to be installed in your network topology.

```
sudo apt-get install ros-$(rosversion -d)-crop-central-api
```
# 3. Getting started:

If this is your first interaction with the CROP package, please follow this section.

## 3.1 CROP components: 

### 3.1.1 Middleware:

The CROP solution is built by a distributed collection of middlewares each in a machine of your insfraestructure topology.

The middleware manages robot requests to complete a task by deploying network applications as VNF's (Virtual network functions). Middleware's speak to one another to deploy reources and perform resource allocation and task optimization & management.

 The core principles and functionality of the Middleware cover the lifecycle management, recovery and error handling of the network applications, and integration of the semantic planning into the orchestration process.

Full documention of the [Middleware](Middleware\readme.md).

### 3.1.2 Network applications & relay.
- The aim of a network application under the CROP 
 frame is to transition local ROS applications to cloud enviroment.

- A network application is a vertical application managed by ROS (Robot Operating System). 

**Why network appliations:** some vertial applications for robotics are heavy and demanding on CPU & GPU, like object detection. If this task can be offloaded to the cloud, the robots have more capabilities in running tasks that are limited by hw specs.

**Traditional ROS PARADIGM:**

<img src="NetApp\img\netapp1.png" alt="drawing" width="200%" height="100%"/>

**CROP CLOUD BASED PARADIGM:**

<img src="NetApp\img\netapp2.png" alt="drawing" width="200%" height="100%"/>

CROP provides a list of already made network applications for usage under ROS 2 in your cloud enviroment. **However, a tutorial for make custom network applications in [available]().** 

Full documentation of the **[network application](NetApp\readme.md).**

Youtube tutorials **[available](https://youtu.be/IhQ1EzOEsOs?si=Ny4BifeUCKInS8Ad&t=2363).** 

### 3.1.3 Dashboard.

The dashboard is the visualization interface for monitoring and control. Each middleware deploys a container with the dashboard.

Access to the dashboard using web browser on local host and port XX. Make sure the middleware is running on the local machine.

```
http://localhost:8080/
```

For more details about the dashboard, follow [here](Dashboard\readme.md).

## 3.2 User roles.

The CROP system is a cloud native solution that involves multiple roles, each assuming an specific user-knowledge. If you are a single ROS developer, you will need to assume all basic roles except CROP developer.

### 3.2.1 Administrator

CROP as a distributed system solution requires an administrator of the middleware instances in the topology for installation, management and support.

### 3.2.2 CROP developer

### 3.2.3 Robot developer

### 3.2.4 Network application Developer

## 3.3 System provision & onboarding

### 3.3.1 Deployment of middlewares & centralAPI.

### 3.3.2 Accounts & Profiling - user onboarding.

### 3.3.3 Robot fleet onboarding.

### 3.3.4 Network app onboarding & task definition.

### 3.4 Task deployment into the robot.

### 3.4.1 Connect robot to local middleware.

### 3.4.2 Task deployment & network app consumption

## 4. Advanced topics.

### 4.1 5G-Testbed integration & slice management.

### 4.2 Middleware policies.

## 5. Useful learning resources.

### 5.1 Youtube channel and tutorials.

You can find the official Youtube channgel [here](https://www.youtube.com/@5g-era460).
There are four workshops with tutorials about the different CROP components.

### 5.2 Online tutorials

### 5.3 ICRA 2024 workshops.

## 6. Social Media and contact information.

For general enquiries you may contact us at:

* https://5g-era.eu/contact/

For social media updates on the latest development and news:
----
* [Facebook](https://www.facebook.com/5geraproject/?ref=pages_you_manage)

* [Twitter](https://twitter.com/5g_era) 


* [Linkedin](https://www.linkedin.com/company/5g-era-project/%20)

---





[1]: https://twitter.com/5g_era

