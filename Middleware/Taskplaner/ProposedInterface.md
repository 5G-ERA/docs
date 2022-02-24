# Endpoints

## Deployment

### POST /deployment/ns/{name} 
Deploy the Network Service with specified name
Param: name - Name of the NS
Return: ID of deployed NS as GUID
Status: 201 (Created), 204 (No Content), 400 (Bad Request)

## GET /deployment/ns/ 
Get all the NS instances
Param: None
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

### GET /deployment/ns/{name} 
Get the NS instances for the specific NS
Param: name - Name of the NS
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
Param: id - ID Of the NS
Status: HTTP 204 when success, 404 when could not find the resource

### POST /deployment/knf/{name} 
Deploy the Kubernetes Network Function with specified name
Param: name - name of the KNF
Return: Id of created KNF
Status: 201 (Created), 204 (No Content), 400 (Bad Request)

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
Param: name - Name of the KNF
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

### DELETE /deploymnet/knf/{ID} 
Delete the NS instance with specified ID
Param: id - ID Of the NS
Status: HTTP 204 when success, 404 when could not find the resource

## Action

### POST /action/execute/{plan} 
Executes resources for the specified plan
Param: plan - List of resources to be instanciated
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
Return:
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
Status: 201 (Created), 204 (No Content), 400 (Bad Request)

### GET /action/{plan_id} 
Get the status of services deployed with this plan
Param: id - GUID id of the plan
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
Status: 200 (OK), 404(Not Found)

### PATCH /action/execute/{plan}
Modifies the existing plan to deploy additional services or remove unnecesary ones
Param: plan - List of resources to be instanciated
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
Status: 200(OK), 204(No Content), 409(Conflict)
### DELETE /action/{PlanId}
Terminates the action plan with the specified Id
Param: PlanId - unique identifier of the plan
Result:

Status: HTTP 204 when success, 404 when could not find the resource

## Additional functionality
* Instanciation of Redis and Middleware and its termination
