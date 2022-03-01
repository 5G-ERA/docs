# Endpoints

This is the proposed endpoint structure for the Orchestrator. The endpoints should allow for the communication with the necessary features of the OSM and apply the plans for the 5G-ERA Middleware.

The full specification of the OSM API can be seen under this [link](https://forge.etsi.org/swagger/ui/?url=https%3A%2F%2Fosm.etsi.org%2Fgitweb%2F%3Fp%3Dosm%2FSOL005.git%3Ba%3Dblob_plain%3Bf%3Dosm-openapi.yaml%3Bhb%3DHEAD).

# Deployment

### POST /deployment/ns/{name} 
Deploy the Network Service with specified name

**Param**: name - Name of the NS

**Return**: ID of deployed NS as GUID

**Status**: 201 (Created), 204 (No Content), 400 (Bad Request)

### GET /deployment/ns/ 
Get all the NS instances

**Param**: None

**Return**: 
```json
[
  {
    "ID": "guid",
    "name": "name",
    "status": "status"
  }
]
```
**Status**: 200 (OK), 404(Not Found)

### GET /deployment/ns/{name} 
Get the NS instances for the specific NS

**Param**: name - Name of the NS

**Return**: 
```json
[
  {
    "ID": "guid",
    "name": "name",
    "status": "status"
  }
]
```
**Status**: 200 (OK), 404(Not Found)

### GET /deployment/ns/{id} 
Get the NS instance by its ID
Param: id - Id of the NS
Return: 
```json
[
  {
    "ID": "guid",
    "name": "name",
    "status": "status"
  }
]
```

Status: 200 (OK), 404(Not Found)

### DELETE /deploymnet/ns/{ID}
Delete the NS instance with specified ID

**Param**: id - ID Of the NS

**Status**: HTTP 204 (OK), 404 (Not found)

### POST /deployment/knf/{name} 
Deploy the Kubernetes Network Function with specified name

**Param**: name - Name of the KNF

**Return**: Id of created KNF

**Status**: 201 (Created), 204 (No Content), 400 (Bad Request)

### GET /deployment/knf/
Get all the KNF instances
Param: 
Return: 
```json
[
  {
    "ID": "guid",
    "name": "name",
    "status": "status"
  }
]
```
Status: 200 (OK), 404(Not Found)

### GET /deployment/knf/{name}
Get the KNF instances for the specific KNF

**Param**: name - Name of the KNF

**Return**: 
```json
[
  {
    "ID": "guid",
    "name": "name",
    "status": "status"
  }
]
```
**Status**: 200 (OK), 404(Not Found)

### DELETE /deploymnet/knf/{ID} 
Delete the NS instance with specified ID

**Param**: id - ID Of the NS

**Status**: HTTP 204 (OK), 404 (Not Found)

# Action

Actions in the understanding of the 5G-ERA are the steps that the robot will have to conduct to finish the specified task.

### POST /action/plan/{plan} 
Executes resources for the specified plan
Asking: Action Planner Response: Orchestrator
**Param**: plan - List of resources to be instantiated
```json
{
  "TaskId": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "Placement: "EDGE/CLOUD",
      "ActionPriority": 1/2/3,
      "Services": [
        "ServiceId/Image name": "Object detection service",
        "ServiceInstanceId": "guid",
        "ServiceType: "Object detection/SLAM",
        "IsResusable": true,        
        "DesiredStatus": "created",        
        "ServiceUrl": "https://...../......",
        "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
      ]      
   } 
  ]
}

```
**Return**:
```json
{
  "TaskId": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "Placement: "EDGE/CLOUD",
      "ActionPriority": 1/2/3,
      "Services": [
        "ServiceId/Image name": "Object detection service",
        "ServiceInstanceId": "guid",
        "ServiceType: "Object detection/SLAM",
        "IsResusable": true,        
        "DesiredStatus": "created",        
        "ServiceUrl": "https://...../......",
        "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
      ]
   } 
  ]
}
```
**Status**: 201 (Created), 204 (No Content), 400 (Bad Request)

### POST /action/plan/{plan} 
**TODO**: retink if needed

Updates the specified plan based on the change in the statuses of the services in the existing plan

**Param**: plan - List of resources to be instantiated
```json
{
  "PlanId": "guid",
  "name": "name",
  "status": "status",
  "steps": [
    {
      "Order": 1,
      "InstanceName": "name",
      "IsResusable": true,
      "DesiredStatus": "created"
    }
  ]
}
```
**Return**:
```json
[
  {
    "Order": 1,
    "InstanceName": "name",
    "InstanceId": "id",
    "Status" : "Status"
  }
]
```
**Status**: 201 (Created), 204 (No Content), 400 (Bad Request)


### GET /action/{plan_id} 
Get the status of services deployed with this plan

**Param**: id - GUID id of the plan

**Return**:
```json
[
   {
      "ActionId": 2,
      "Order": 0,
      "Placement: "EDGE/CLOUD",
      "ActionPriority": 1/2/3,
      "Services": [
        "ServiceId/Image name": "Object detection service",
        "ServiceInstanceId": "guid",
        "ServiceType: "Object detecction/SLAM",
        "IsResusable": true,        
        "DesiredStatus": "created",        
        "ServiceUrl": "https://...../......",
        "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
      ]
   } 
]
```
Status: 200 (OK), 404(Not Found)

### PATCH /action/execute/{plan}
Modifies the existing plan to deploy additional services or remove unnecessary ones

**Param**: plan - List of resources to be instantiated
```json
{
  "PlanId": "guid",
  "name": "name",
  "status": "status",
  "steps": [
    {
      "Order": 1,
      "InstanceName": "name",
      "IsResusable": true,
      "DesiredStatus": "terminated"
    }
  ]
}
```
Return:
```json
[
  {
    "Order": 1,
    "InstanceId": "id",
    "Status": "status"
  }
]
```
**Status**: 200(OK), 204(No Content), 409(Conflict)
### DELETE /action/action/{actionId}
Terminate services for specified action in a plan

Param:
```json
{
  "TaskId": "TASK_NUMBER",  
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,      
      "Placement: "EDGE/CLOUD",      
      "Services": [
        "ServiceId/Image name": "Object detection service",
        "ServiceInstanceId": "guid",
        "IsResusable": "true"        
      ]
   } 
  ]
}
```
**Status**: HTTP 204 (OK), 404 (Not Found)

### DELETE /action/{PlanId}
Terminates the action plan with the specified Id

**Param**: PlanId - unique identifier of the plan

**Status**: HTTP 204 (OK), 404 (Not Found)

## Additional functionality
* Instantiation of Redis and Middleware and its termination during the startup and shutdown
