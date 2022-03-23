# Endpoints

This is the proposed endpoint structure for the Orchestrator. The endpoints should allow for the communication with the necessary features of the OSM and apply the plans for the 5G-ERA Middleware.

The full specification of the OSM API can be seen under this [link](https://forge.etsi.org/swagger/ui/?url=https%3A%2F%2Fosm.etsi.org%2Fgitweb%2F%3Fp%3Dosm%2FSOL005.git%3Ba%3Dblob_plain%3Bf%3Dosm-openapi.yaml%3Bhb%3DHEAD).

## Orchestrate

Actions in the understanding of the 5G-ERA are the steps that the robot will have to conduct to finish the specified task.

### POST /orchestrate/plan/{plan}

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

**Status**: 201 (Created), 204 (No Content), 400 (Bad Request)

### GET /orchestrate/plan/{plan_id}

Get the status of services deployed with this plan

**Param**: id - GUID id of the plan

**Return**:

```json
[
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
```

Status: 200 (OK), 404(Not Found)

### PATCH /orchestrate/execute/{plan}

Modifies the existing plan to deploy additional services or remove unnecessary ones

**Param**: plan - List of resources to be instantiated

```json
{
  "Id": "guid",
  "name": "name",
  "status": "status",
  "steps": [
    {
      "Order": 1,
      "InstanceName": "name",
      "IsReusable": true,
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

### DELETE /orchestrate/action/{actionId}

Terminate services for specified action in a plan

Param:

```json
{
  "Id": "TASK_NUMBER",  
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,      
      "Placement": "EDGE/CLOUD",      
      "Services": [
        {
          "Id": "guid",
          "Image name": "Object detection service",
          "ServiceInstanceId": "guid",
          "IsReusable": "true"   
        }
      ]
   } 
  ]
}
```

**Status**: HTTP 204 (OK), 404 (Not Found)

### DELETE /orchestrate/plan/{PlanId}

Terminates the action plan with the specified Id

**Param**: PlanId - unique identifier of the plan

**Status**: HTTP 204 (OK), 404 (Not Found)

## Additional functionality

* Instantiation of Redis and Middleware and its termination during the startup and shutdown
