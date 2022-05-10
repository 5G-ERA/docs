# External Interface
This document presents the endpoints that are accessible in the middleware. This document covers the path to the endpoints, what is their functionality, what parameters they take and what information is returned alongside the status codes. 

All the endpoints except the authentication ones require the client to be authenticated. The authentication scheme utilizes the JWT Bearer token. The token is retrieved after logging in using the `/Login` endpoint. The token has to be passed in the `Authentication` header for each request to be allowed.

## Authentication

### POST /Register
Registers the robot into the Middleware system. Task to be performed by the Administrator.

Param:
```json 
{
   "Id":"3cace17e-a091-4f91-ad98-7d0c371b3a25",
   "Password":"password"
}
```
Return:
* 201 - after successful registration
* 400 - when parameters are not specified
* 500 - when the server experiences an error


### POST /Login
Authenticates the Robot in the system. When authenticated, the Robot is given the JWT Token to be used as a Bearer authentication scheme thought the usage of the system.
Param:
```json 
{
   "Id":"3cace17e-a091-4f91-ad98-7d0c371b3a25",
   "Password":"password"
}
```
Return:
* 200 - user has ben authenticated. Return body:
```json
{
     "token":"JWTBearerToken",
     "expirationDate": "2022-05-09T22:35:56.8472623Z"
}
```
* 400 - Credentials were not provided
* 401 - User have not ben authorized, wrong credentials

## Task planner
Task planner is responsible for the creation of the task and its initialization in the Middleware. It handles the full pipeline, from the given taskId for the execution to the deployment of the task.

### GET /PLAN/{param}

Asking: ROBOT ANSWER: 5G-ERA.

(Assumptions: robot knows the task_id for a high level action for example `grab the medicine`).

The ROBOT asks for help from 5G-ERA system. 

Param: 
```json
{
  "Id": "guid",
  "TaskId" : "task_id",
  "Questions": [
    {
    "Id": "guid",
    "Question": "Do you have a map?",
    "Answer": "True/false"
    }
  ]
}
```

Return: 
* 200 - the status of the plan to be deployed, including the latest deployment information of the actions in the task. The example body:

```json
{
  "Id": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "Id": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
        "ServiceType": "Object detection/SLAM",   
        "ServiceUrl": "https://...../......",
        "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]
   } 
  ]
}
```
* 404 - the plan definition has not been found


## Orchestrator

This is the proposed endpoint structure for the Orchestrator. The endpoints should allow for the communication with the necessary features of the OSM and apply the plans for the 5G-ERA Middleware.

Actions in the understanding of the 5G-ERA are the steps that the robot will have to conduct to finish the specified task.

### POST /orchestrate/orchestrate/plan/{plan}

Executes resources for the specified plan
Asking: Action Planner Response: Orchestrator

**Param**: plan - List of resources to be instantiated
```json
{
  "Id": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "Id": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
          "Id": "guid",
          "Image name": "Object detection service",
          "ServiceInstanceId": "guid",
          "ServiceType": "Object detection/SLAM",
          "IsReusable": true,
          "DesiredStatus": "created",        
          "ServiceUrl": "https://...../......",
          "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]      
   } 
  ]
}

```

**Return**:
* 200 - plan has ben updated and instantiated. Returns the body:
```json
{
  "Id": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
          "Id": "guid",
          "Image name": "Object detection service",
          "ServiceInstanceId": "guid",
          "ServiceType": "Object detection/SLAM",
          "IsReusable": true,
          "DesiredStatus": "created",        
          "ServiceUrl": "https://...../......",
          "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]
   } 
  ]
}
```
* 400 - the plan to be deployed have not been specified
* 500 - The error occurred while deploying the services

### GET orchestrate/orchestrate/plan/{plan_id}

Get the status of services deployed with this plan

**Param**: id - GUID id of the plan

**Return**:
* 200 - the status of the services have been retrieved. The body with the statuses:
```json
{
    "ActionPlanId":"50a85398-21b5-4604-8c6f-d510d2abbfd3",
    "ActionSequence":[
    {
      "ActionId": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
          "Id": "guid",
          "Image name": "Object detection service",
          "ServiceInstanceId": "guid",
          "ServiceType": "Object detection/SLAM",
          "IsReusable": true,
          "DesiredStatus": "created",        
          "ServiceUrl": "https://...../......",
          "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]
    }]
}
```
* 400 - the plan to be deployed have not been specified
* 404 - the plan with the specified Id have not been found
* 500 - The error occurred while deploying the services

Status: 200 (OK), 404(Not Found)

### DELETE /orchestrate/action/{id}

Terminate services for specified action plan.

Param:
* **id** - the Identifier of the action plan to be deleted

Returns:
* 200 - plan has been deleted
* 400 - the Id of the plan has not been specified
* 404 - The plan has not been found
* 500 - There was an error while deleting the plan
