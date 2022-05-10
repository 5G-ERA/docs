# Proposed API endpoints for RedisAPI
Assumption for each of the end points we can get all and get by specific ID.
Assumption that all policies are general to all robots.

## REDISINTERFACE

RedisInterface API allows full CRUD operations (GET/POST/PUT/PATCH/DELETE) on all data models that are embebed in the system. 

### GET Data/Action
Get all the actions from redis.

Return:

* 200 - list of Actions from redis storage
```json 
[
  {
    "Id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "Name": "move-base",
    "ActionFamily": "movment",
    "Order": 10,
    "Placement": "cloud",
    "ActionPriority": "medium"
  },
  {
    "Id": "425b46eb-eaea-45e0-8769-4926159f9c87",
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "edge",
    "ActionPriority": "low"
  }
  
]
```
* 404 - Actions were Not found
* 500 - Server Error

### POST Data/Action
Add new action entity in redis.

Param:
```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "ActionFamily": "string",
  "Order": 0,
  "Placement": "string",
  "ActionPriority": "string"
}
```

Return:

* 200 - Newly created action entity.
```json 
{
    "Id": "425b46eb-eaea-45e0-8769-4926159f9c87",
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "edge",
    "ActionPriority": "low"
}
```
* 400 - Bad Request parameters were not specified corectly.
* 500 - Server Error.

## ORCHESTRATOR

Full CRUD operations on services statuses. It stores the status of all the services deployed by the OSM.
Service definition: knf/vnf --> container with an image and some functions.
Instance definition: network service deployed by the OSM/k8. Knf/Vnf deployed.
Action definition: step to be conducted to execute a task - actionSequence. - Individual action from action sequence.
Task: high level goal of robot.

### GET/POST/DELETE/PATCH /instance/{instance info}

ASKING: Orchestrator ANSWERING: RedisAPI

Param:

* GET/DELETE - instance Id
* POST/PATCH - instance definition

```json
{
    "Id": "Service_Number",
    "ImageName": "Object-detection-service",
    "ServiceInstanceId": "guid",
    "ServiceType": "Object detection/SLAM",
    "IsReusable": true,
    "DesiredStatus": "created",        
    "ServiceUrl": "https://...../......",
    "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
}
```

### GET/DELETE/POST /instances/action/{}

Param:

* GET/DELETE: ActionId, InstanceId
* POST/PATCH: ActionId, InstanceId

Param:

```json
{
    "ActionId" : "guid",
    "InstanceId": "guid"
}
```

### GET/POST/DELETE/PATCH /action/{}

Perform the CRUD operation on current statuses of the actions:
Param:

* GET/DELETE: ActionId, TaskId/ActionPlanId
* POST/PATCH: ActionId, InstanceId

return:

```json
{    
    "Id": 2,
    "ActionPlanId/TaskId": "guid",
    "Order": 0,
    "Placement": "EDGE/CLOUD",
    "ActionPriority": "1/2/3",
}
```

### GET/POST/DELETE/PATCH /image/{}

Perform get or update operations to an image:
Param:

* GET/DELETE: ImageId
* POST/PATCH: ImageId, ImageInstance

return:

```json
{    
    "Id": "ImageId",
    "RepositoryName": "RepositoryName",
    "Tag": "Tag"
}
```

## TASK PLANNER

### GET /POLICY/Current

Asking: RESOURCE PLANNER ANSWER: SEMANTIC DB REDIS. Function to get the running policies 

Return:

```json
[
  {
    "Id": 10,
    "PolicyName": "PolicyName",
    "PolicyDescription": "lorem ipsum"    
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /POLICY/All

Asking: RESOURCE PLANNER ANSWER: SEMANTIC DB REDIS. Function to get the running policies.

Return:

```json
[
  {
    "Id": 10,
    "PolicyName": "PolicyName",
    "PolicyDescription": "lorem ipsum",
    "IsActive": "true/false",
    "Timestamp": "dd/MM/yyyy"
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /ROBOTS/ {all}

Get all the information about all robots in the middleware.

Asking: DASHBOARD ANSWER: REDIS API.

Return:

```json
[
  {
    "Id": 10,
    "RobotStatus": "Running/withOutBattery",
    "CurrentTaskID": "Task_Number",
    "TaskList": ["Task_Number"],
    "BatteryStatus": 90,
    "LocomotionSystem": "Ackerman/differential_Drive",
    "Sensors": ["lidar", "camera", "IMU"],
    "CPU": 90,
    "RAM": 90,
    "VirtualRam": 90,
    "StorageDisk": 90,
    "NumberCores": 3,
    "Questions": [
        {"map": "present, none"},
        {"Question":"Answer"}
    ]
    
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /EDGES/ {all}

Get all the information about all edges in the middleware.

Asking: DASHBOARD ANSWER: REDIS API.

Return:

```json
[
  {
    "Id": 10,
    "EdgeStatus": "Running/withOutBattery",
    "EdgeIp": "192.168.1.2",
    "CPU": 90,
    "RAM": 90,
    "VirtualRam": 90,
    "StorageDisk": 90,
    "NumberCores": 3
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /CLOUD/ {all}

Get all the information about all clouds in the middleware.

Asking: DASHBOARD ANSWER: REDIS API.

Return:

```json
[
  {
    "Id": 10,
    "CloudStatus": "Running",
    "CloudIp": "192.168.1.2"
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /TASK/ {all}

Get all the information about all tasks.

Asking: DASHBOARD ANSWER: REDIS API.

Return:

```json
[
  {
      "Id": "TASK_NUMBER",
      "TaskName": "TaskName",
      "TaskDescription": "Lorem Ipsum",
      
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /task/robot/{}

Return all the tasks executed by all robots.

Return:

```json

{
    "TaskId" : "TaskId",
    "RobotID": "RobotID"
}
```

### GET /GRAPH_TOPOLOGY/{all}

**TODO**: Discuss in more datail the communication
Function to get a GRAPH.

RESOURCE PLANNER ANSWER: REDIS GRAPH
Param: id - Id of the NS

```json
[
  {
    "REDIS_GRAPH_QUERY": "GRAPH",
  }
]
```

Return:

```json
[
  {
    "UPDATED_GRAPH": "GRAPH",
  }
]
```

Status: 200 (OK), 404(Not Found)

## RESOURCE PLANNER

### GET/POST/DELETE/PATCH /QoE/{}

Get/ add QoE metrics to a tasks performed by robot.
Param:

* GET/DELETE: TaskId
* POST/PATCH: TaskId, QoE

return:

```json
{    
    "Id": "TaskId",
    "QoE": 10,
    "QoEDescription": "Lorem Ipsum"
}
```

### GET/POST/DELETE/PATCH /QoS/{}

Get/ add QoS metrics to a tasks performed by robot.
Param:

* GET/DELETE: ActionId, TaskId/ActionPlanId
* POST/PATCH: ActionId, InstanceId

return:

```json
{    
    "Id": "ActionID",
    "QoS": 10,
    "QoSDescription": "Lorem Ipsum"
}
```

## AUTHENTICATION

Get or update the credentials of the robot.
Param:

* GET/DELETE: UserID, UserName
* POST/PATCH: UserID, Password, UserName

return:

```json
{    
    "UserName": "UserName",
    "Id": "UserID",
    "Password": "Password"
}
```

## AUTHORIZATION

TODO
