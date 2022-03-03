# Proposed API endpoints for RedisAPI

# Orchestrator

Full CRUD operations on services statuses. It stores the status of all the services deployed by the OSM.
Service definition: knf/vnf --> container with an image and some functions.

### GET/POST/DELETE/PATCH /instance/{instance info}

Param: 
* GET/DELETE - instance Id
* POST/PATCH - instance definition

```json
{
    "ServiceId/Image name": "Object detection service",
    "ServiceInstanceId": "guid",
    "ServiceType": "Object detection/SLAM",
    "IsReusable": true,
    "DesiredStatus": "created",        
    "ServiceUrl": "https://...../......",
    "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
}
```


### GET/POST/DELETE/PATCH /instances/action/{}
Perform the CRUD operation on association of actions with the specified instances:
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
Perform the CRUD operation on current statuses of the clouds:
Param: 
* GET/DELETE: ActionId, TaskId/ActionPlanId
* POST/PATCH: ActionId, InstanceId

Param:
```json
{    
    "ActionId": 2,
    "ActionPlanId/TaskId": "guid",
    "Order": 0,
    "Placement": "EDGE/CLOUD",
    "ActionPriority": "1/2/3",
}
```
