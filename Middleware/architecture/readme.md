# Middleware architecture

This document describes the architecture used by the Middleware. How it interacts with other system components and backing resources needed like databases or platforms it is designed to run on. 

Diagram below presents a very high level visualization of the Middleware architecture.

![middleware architecture](../diagrams/crop-architecture.png)

The main components of the Middleware are:

* Gateway - It redirects the traffic across the Middleware system meaning rerouting to the microservices within the system. It also handles the authentication and authorization process.

* Task Planner - Integrating the semantic knowledge of the vertical into resource planning. It is part of the vertical-level life cycle management implemented by Middleware.

* Resource Planner - is responsible for assigning the placement example, on the cloud, Edge to the tasks.

* Orchestrator - it orchestrates the deployment process of resources. It is responsible for the vertical-level lifecycle management of the deployed services

* Redis Interface - allows the users to retrieve, insert, and update data from/into the Redis-Server
ROS: robotics framework 

## Infrastructure provision

Currently, the required infrastructure to deploy 5G-ERA Middleware can be deployed under the AWS private cloud using the dedicated Terraform module. More information about how to run the module and what it deploys can be found in the [middleware repository](https://github.com/5G-ERA/middleware/tree/main/terraform/AWS).