# Proposed endpoints for RedisInterface API
Assumption for each of the end points we can get all and get by specific ID.
Assumption that all policies are general to all robots.

## REDISINTERFACE

RedisInterface API allows full CRUD operations (GET/POST/PUT/PATCH/DELETE) on all data models that are embebed in the system. 

### GET Data/Action
Get all the action objects.

Return:

* 200 - Success, list of Actions.
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
* 404 - Objects were not found.
* 500 - An error occurred.

### POST Data/Action
Add new action entity.

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

* 200 - Success, newly created action entity.
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
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Action/{id}
Get action entity by id.

Param:

* id -  provide the guid id for the desired action entity to be retrieved ex. 425b46eb-eaea-45e0-8769-4926159f9c87.

Return:

* 200 - Success, the specified action entity.
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
* 404 - Object was not found.
* 500 - An error occurred.

### PATCH Data/Action/{id}
Modify an existing action entity.

Param:

* id - provide the guid id for the desired action entity to be modified ex. 425b46eb-eaea-45e0-8769-4926159f9c87.

```json
{
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "cloud",
    "ActionPriority": "high"
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
    "Id": "425b46eb-eaea-45e0-8769-4926159f9c87",
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "cloud",
    "ActionPriority": "high"
}
```
* 404 - Action entity was not found.
* 500 - An error occurred.

### DELETE Data/Action/{id}
Delete an action entity.

Param:

* id -  provide the guid id for the desired action entity to be deleted ex. 425b46eb-eaea-45e0-8769-4926159f9c87.

Return:

* 200 - Success, true.
* 404 - Action entity was not found.
* 500 - An error occurred.

### POST Data/Action/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "type": "Action",
    "name": "move-base"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "type": "Action",
    "name": "move-base"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  }
}
```
* 400 - Bad Request parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Action/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired action entity relation to be retrieved ex. e5ebd4bbe-9c75-46df-a4f4-93c7dea10373.
* name - provide the relation name ex. NEEDS.

Return:

* 200 - Success, relation was found.
```json
{
  "initiatesFrom": {
    "id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "type": "Action",
    "name": "move-base"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Action/relations/{firstName}/{secondName}
Currently, Action has single relation use case scenario.

### GET Data/Action/plan
Get all the plans.

Return:

* 200 - Success, plans were found.
```json
[
  {
    "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
    "name": "TestTask",
    "actionSequence": [
      {
        "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
        "name": "move-base",
        "actionFamily": "movement",
        "order": 1,
        "placement": "cloud",
        "actionPriority": "low",
        "services": null,
        "relations": []
      },
      {
        "id": "7e085e54-2c67-473a-9da9-732469d265bf",
        "name": "detect-botle",
        "actionFamily": "detection",
        "order": 2,
        "placement": "cloud",
        "actionPriority": "medium",
        "services": null,
        "relations": []
      }
    ],
    "relations": []
  }
]
```
* 404 - Plans were not found.
* 500 - An error occurred.

### POST Data/Action/plan
Add new plan.

Param:

```json
{
  "Id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "Name": "TestTask",
  "ActionSequence": [
    {
      "Id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "Name": "move-base",
      "ActionFamily": "movement",
      "Order": 1,
      "Placement": "cloud",
      "ActionPriority": "low"
    },
     {
      "Id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "Name": "detect-botle",
      "ActionFamily": "detection",
      "Order": 2,
      "Placement": "cloud",
      "ActionPriority": "medium"
    }
  ]
}
```

Return:

* 200 - Success, plan was created.
```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```
* 400 - Bad Request parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Action/plan/{id}
Get plan by id.

Param:

* id - provide the guid id for the desired action plan to be retrieved ex. 97bb19b1-042c-408e-b1d9-2a2ebbe05b66.

Return:

* 200 - Success, plan scuccessfully retrieved.
```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```
* 404 - Plans were not found.
* 500 - An error occurred.

### DELETE Data/Action/plan/{id}
Delete plan by id.

Param:

* id - provide the guid id for the desired action plan to be deleted ex. 97bb19b1-042c-408e-b1d9-2a2ebbe05b66.

Return:

* 200 Success, plan was deleted.
* 404 Plan was not found.
* 500 An error occurred.

### PUT Data/Action/plan/{id}
Update an existing plan.

Param:

* id - provide the guid id for the desired action plan to be updated ex. 97bb19b1-042c-408e-b1d9-2a2ebbe05b66.

```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```

Return:

* 200 Success, the plan was updated.
```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```
* 400 Bad Request parameters were not specified correctly.
* 404 The plan to be uodated was not found.
* 500 An error occurred.

### GET Data/Cloud
Get all the clouds.

Return:

* 200 - Success, list of Clouds from redis storage.
```json
[
  {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "name": "Cloud_1",
    "cloudStatus": "Running",
    "cloudIp": "192.168.1.1",
    "relations": []
  },
  {
    "id": "186e9c60-4682-4ec0-85b4-9284f39a381d",
    "name": "Cloud_2",
    "cloudStatus": "Running",
    "cloudIp": "192.168.1.2",
    "relations": []
  }
]
```
* 404 - Clouds were not found.
* 500 - An error occurred.

### POST Data/Cloud
Add new cloud.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "CloudStatus": "string",
  "CloudIp": "string"
}
```

Return:

* 200 Success, newly created cloud entity
```json
{
  "id": "186e9c60-4682-4ec0-85b4-9284f39a381d",
  "name": "Cloud_2",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.2",
  "relations": []
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Cloud/{id}
Get cloud by id.

Param:

* id - provide the guid id for the desired cloud entity to be retrieved ex. e56e1928-03a0-43e9-b999-318198c6d7ca.

Return:

* 200 - Success, the specified entity.
```json
{
  "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
  "name": "Cloud_1",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.1",
  "relations": []
}
```
* 404 - Cloud was not found.
* 500 - Server Error.

### PATCH Data/Cloud/{id}
Modify an existing cloud entity.

Param:

* id - provide the guid id for the desired cloud entity to be modified ex. e56e1928-03a0-43e9-b999-318198c6d7ca.

```json
{
  "name": "Cloud_1",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.1",
  "relations": []
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
  "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
  "name": "Cloud_1",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.5",
  "relations": []
}
```
* 404 - Cloud entity was not found.
* 500 - An error occurred.

### DELETE Data/Cloud/{id}
Delete a cloud entity.

Param:

* id - provide the guid id for the desired cloud entity to be deleted ex. e56e1928-03a0-43e9-b999-318198c6d7ca.

Return:

* 200 - Success, cloud was deleted.
* 404 - Cloud entity was not found.
* 500 - An error occurred.

### POST Data/Cloud/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "type": "Cloud",
    "name": "Cloud_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "type": "Cloud",
    "name": "Cloud_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Cloud/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired Cloud entity relation to be retrieved ex. e56e1928-03a0-43e9-b999-318198c6d7ca.
* name - provide the relation name ex. WORKING_FOR.

Return:

* 200 - Success, relation was found.
```json
{
  "initiatesFrom": {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "type": "Cloud",
    "name": "Cloud_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Cloud/relations/{firstName}/{secondName}
Currently, Cloud has single relation use case scenario.

### GET Data/ContainerImage
Get all containers.

Return:

* 200 - Success, list of containers from redis storage.
```json
[
  {
    "id": "1ae73495-01f3-4273-8fd7-003c941a8491",
    "name": "my-image-8",
    "timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "description": "my-description-8",
    "k8SDeployment": "my-deployment-8",
    "k8SService": "mk8s",
    "relations": []
  },
  {
    "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
    "name": "my-image-9",
    "timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "description": "my-description-9",
    "k8SDeployment": "my-deployment-9",
    "k8SService": "mk8s",
    "relations": []
  }
]
```
* 404 - Containers were not found.
* 500 - An error occurred.

### POST Data/ContainerImage
Create a new container entity.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "Timestamp": "2022-05-12T14:22:17.702Z",
  "Description": "string",
  "K8SDeployment": "string",
  "K8SService": "string"
}
```

Return:

* 200 - Success, newly created container entity.
```json 
{
  "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "mk8s",
  "relations": []
}
```
* 400 - Bad Request parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/ContainerImage/{id}
Get container by id.

Param:

* id - provide the guid id for the desired container entity to be retrieved ex. 92e97d88-d7f1-4d17-959f-b7e03083a547.

Return:

* 200 - Success, the specified entity.
```json
{
  "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "mk8s",
  "relations": []
}
```
* 404 - ContainerImage entity was not found.
* 500 - An error occurred.

### PATCH Data/ContainerImage/{id}
Modify an existing container entity.

Param:

* id - provide the guid id for the desired container entity to be modified ex. 92e97d88-d7f1-4d17-959f-b7e03083a547.

```json
{
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "kubernetes",
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
   "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "kubernetes",
  "relations": []
}
```
* 404 - ContainerImage entity was not found.
* 500 - An error occurred.

### DELETE Data/ContainerImage/{id}
Delete a container entity.

Param:

* id -  provide the guid id for the desired container entity to be deleted ex. 92e97d88-d7f1-4d17-959f-b7e03083a547.

Return:

* 200 - Success, container was deleted.
* 404 - ContainerImage entity was not found.
* 500 - An error occurred.

### POST Data/ContainerImage/AddRelation
Currently the ContainerImage does not support relations to initiate from.

### GET Data/ContainerImage/relation/{name}
Currently the ContainerImage does not support relations to initiate from.

### GET Data/ContainerImage/relations/{firstName}/{secondName}
Currently the ContainerImage does not support relations to initiate from.

### GET Data/ContainerImage/instace/{id}
Get the container images associated with the specified instance.

Param:

* id - provide the guid id for the instance entity associated with the container images ex. d3f31c7e-8b6c-43d7-9cb5-99838ed27880.

Return:

* 200 Success list of container images
```json
[
  {
    "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
    "name": "my-image-9",
    "timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "description": "my-description-9",
    "k8SDeployment": "my-deployment-9",
    "k8SService": "mk8s",
    "relations": []
  }
]
```
* 404 - Container images were not found.





```json
```
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
