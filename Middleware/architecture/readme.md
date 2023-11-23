# Middleware architecture

This document describes the architecture used by the Middleware. How it interacts with other system components and backing resources needed like databases or platforms it is designed to run on. 


![middleware architecture](../img/middleware_architecture.drawio.png)

5G-ERA Middleware is allowing robots from different vertical sectors to use 5G-based digital
skills to enhance their autonomy. The Middleware is the link between vertical applications
managed by ROS and 5G infrastructure managed by OSM. It realises the 5G-ERA intentbased network using cloud-native design. The Middleware can be instantiated in the core
network either in the Edge Machines or in the cloud. The implementation allows the Robot to
request the instantiation of the cloud-native resources that will support the execution of the
task. The main components of the Middleware are:

* Gateway- It redirects the traffic across the Middleware system meaning rerouting to the microservices within the system. It also handles the authentication and authorization process.

* Task Planner – Integrating the semantic knowledge of the vertical into resource planning. It is part of the vertical-level life cycle management implemented by Middleware.

* Resource Planner – is responsible for assigning the placement example, on the cloud, Edge to the tasks.

* Orchestrator – It orchestrates the process of the deployment of the resources. It is responsible for the vertical-level lifecycle management of the deployed services

* Redis Interface –Allows the users to retrieve, insert, and update data from/into the Redis-Server
ROS: robotics framework 




## Infrastructure provision

Currently the required infrastructure to deploy 5G-ERA Middleware can be deployed under the AWS private cloud using the dedicated Terraform module. More information about how to run the module and what it deploys can be found in teh [middleware repository](https://github.com/5G-ERA/middleware/tree/main/terraform/AWS).