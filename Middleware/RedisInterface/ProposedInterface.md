# Proposed API endpoints for RedisAPI
Assumption for each of the end points we can get all and get by specific ID.

# Orchestrator

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
    "ServiceId": "Service_Number"
    "ImageName": "Object-detection-service",
    "ServiceInstanceId": "guid",
    "ServiceType": "Object detection/SLAM",
    "IsReusable": true,
    "DesiredStatus": "created",        
    "ServiceUrl": "https://...../......",
    "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
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
    "ActionId": 2,
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
    "ImageId": "ImageId",
    "RepositoryName": "RepositoryName",
    "Tag": "Tag"
}
```

# TASK PLANNER 

### GET /POLICY/Current

**TODO**: Move to Redis API

Asking: RESOURCE PLANNER ANSWER: SEMANTIC DB REDIS. Function to get the running policies 

Return: 
```json
[
  {
    "PolicyId": 10,
    "PolicyDescription": "lorem ipsum"    
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /POLICY/All

**TODO**: Move to Redis API

Asking: RESOURCE PLANNER ANSWER: SEMANTIC DB REDIS. Function to get the running policies. 

Return: 
```json
[
  {
    "PolicyId": 10,
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
    "RobotID": 10,
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
    "EdgeID": 10,
    "EdgeStatus": "Running/withOutBattery",
    "EdgeIp": "192.168.1.2"
    "CPU": 90
    "RAM": 90
    "VirtualRam": 90
    "StorageDisk": 90
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
    "CloudID": 10,
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
      "TaskId": "TASK_NUMBER",
      "TaskName": "TaskName"
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

**TODO**: Move to Redis API 
Function to ger a GRAPH. 

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

# RESOURCE PLANNER

### GET/POST/DELETE/PATCH /QoE/{}
Get/ add QoE metrics to a tasks performed by robot.
Param: 
* GET/DELETE: TaskId
* POST/PATCH: TaskId, QoE

return:
```json
{    
    "TaskId": TaskId,
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
    "ActionID": "ActionID",
    "QoS": 10,
    "QoSDescription": "Lorem Ipsum"
}
```

# AUTHENTICATION
Get or update the credentials of the robot.
Param: 
* GET/DELETE: UserID, UserName
* POST/PATCH: UserID, Password, UserName

return:
```json
{    
    "UserName": "UserName",
    "UserID": "UserID",
    "Password": "Password"
}
```
# AUTHORIZATION: 
TODO

