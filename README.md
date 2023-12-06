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
<table style="width: 98%; margin-right: calc(2%);">
    <tbody>
        <tr>
            <td style="width: 19.9372%;"><br></td>
            <td style="width: 19.9372%;">Potential Users</td>
            <td style="width: 19.9686%;">Example</td>
            <td style="width: 19.9686%;">Technical Challenges</td>
            <td style="width: 20.0000%;">CROP solution</td>
            <td style="width: 20.0000%;">Tutorial and Documentation</td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Scenario 1</td>
            <td style="width: 19.9372%;">Robot Application Developers </td>
            <td style="width: 19.9686%;">I want my ROS application to be deployed in the cloud and Interact with my local robots. 

Problem, although my ROS code works fine in LAN, it does not work anymore through the Internet.
</td>
            <td style="width: 19.9686%;">•	The Internet has multiple domains and multi administrations. Various security requirements and package inspection kill my existing ROS application. </td>
            <td style="width: 19.9686%;">CROP integrates network function virtualisation into robot deployment. 

It generates a static virtual LAN for ROS applications, at the same time dynamically adapting the network resources for the virtual network on demand.  

Domain knowledge of Robotics and ICT are encapsulated separately.
</td>
            <td style="width: 19.9686%;">(Pre-required Step1: that is needed to be followed in every scenario mentioned*) Go through the prerequisites and middleware installation tutorial

Step 2: Onboard your custom robot from the onboarding robot tutorial 
Step 3: Onboard the ROS network application from the tutorial 
Step 4: Trigger the Deployment of  your ROS network application
</td>
            <td style="width: 19.9686%;"><br>
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Scenario 2</td>
            <td style="width: 19.9372%;">Robot Application Developers
</td>
            <td style="width: 19.9686%;">I want to give the best possible  resources to my mobile robots. Although, there the answer is not fixed. It could be either 5G or WiFi; either Local Edge or Remote Cloud; either eMBB or URLLC slice.

Problem, the task is cumbersome and completely out of my knowledge. At the end, I just use VPN and hard code everything to the robot. 

</td>
             <td style="width: 19.9686%;">•	Network topology for mobile robots is dynamic. 
•	Knowledge gap between robotics and ICT.
</td>
              <td style="width: 19.9686%;"></td>
               <td style="width: 19.9686%;">Step 2: Register and onboard your robot 
Step 3: Tutorial will be added (Configuring of the infrastructure of the testbed) slice manager
Step 4: Configuration of new or existing tasks for utilising slice mechanism 
Step 5: Trigger the Deployment of  your ROS network application
</td>
            <td style="width: 19.9686%;">
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
    </tbody>
</table>
