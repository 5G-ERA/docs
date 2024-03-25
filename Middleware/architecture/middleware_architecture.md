# Middleware Architecture divided into 5 components;

1. Gateway – It redirects the traffic across the Middleware system meaning rerouting to
the microservices within the system. It also handles the authentication and
authorisation process.

2. Action Planner/Task Planner – Integrating the semantic knowledge of the vertical into resource
planning. It is part of the vertical level life cycle management implemented by
Middleware.

3. Resource Planner – is responsible for assigning the placement example, on the cloud,
Edge to the tasks.

4. Orchestrator – It orchestrates the process of the deployment of the resources. It is
responsible for the vertical level lifecycle management of the deployed services

5. Redis Interface –Allows the users to retrieve, insert and update data from/into the
Redis-Server 
