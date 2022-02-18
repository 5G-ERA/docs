# Endpoints

## Authentication
TBD
### /authentication/
TBD

## Deployment of Network Service

### POST /deployment/ns/{ns_request} 
Initiating a new network service in the OSM
Param:
```json
[
 {
   "NSname": "NSname",
   "configuration":[
     {
       "scale":"1"
       "parameters":"parameters"
     }
    ] 
  }
]
```
Return:
```json
[
  {
    "NSInstanceName": "NSname",
    "NSInstanceId": "NSid",
    "InstanceRef":[
     {
       "url":"url"
       "entrypoint":"entrypoint"
     }
    ] 
    "Status" : "Status"
  }
]
```
Status: 201 (Initilised), 201(Created), 203(Failed), 204 (No Content), 400 (Bad Request)


### GET /deployment/ns/{ns_request} 
Get the information about a deployed service
Param:
```json
[
  {
    "NSInstanceId": "NSid",
    "NSInstanceName": "NSname",
    "last_knownstatus" : "Status"
  }
]
```
Return:
```json
[
  {
    "NSInstanceName": "NSname",
    "NSInstanceId": "NSid",
    "Status" : "status"
    "Feedback" : "feedback"
    "InstanceRef":[
     {
       "url":"url"
       "entrypoint":"entrypoint"
     }
    ] 
    "configuration":[
      {
        "Scale":"1"
        "Parameters":"parameters"
      }
    ]
  }
]
```
### PATCH /deployment/ns/{ns_request} 
Scale up and down of an existing network service
Param:
```json
[
  {
    "NSInstanceId": "NSid",
    "NSInstanceName": "NSname",
    "last_knownstatus" : "Status"
    "configuration":[
     {
       "scale":"1"
       "parameters":"parameters"
     }
    ] 
  }
]
```
Return:
```json
[
  {
    "NSInstanceName": "NSname",
    "NSInstanceId": "NSid",
    "Status" : "status"
    "Feedback" : "feedback"
    "InstanceRef":[
     {
       "url":"url"
       "entrypoint":"entrypoint"
     }
    ] 
    "configuration":[
      {
        "Scale":"scale"
        "Parameters":"parameters"
      }
    ]
  }
]
```
### DELETE /deployment/ns/{ns_request} 
Terminate an existing network service
Param:
```json
[
  {
    "NSInstanceId": "NSid",
    "NSInstanceName": "NSname",
    "last_knownstatus" : "Status"
  }
]
```
Return:
```json
[
  {
    "NSInstanceName": "NSname",
    "NSInstanceId": "NSid",
    "Status" : "Status"
  }
]
```

## Deployment of a Task
### POST /deployment/task/{task_request} 
Initiating a new task in the 5G-ERA
Param:
```json
[
 {
   "name": "name",
   "configuration":[
     {
       "placement":"placement"
       "slicing":"slicing"
     }
    ] 
  }
]
```
Return:
```json
[
  {
    "TaskInstanceName": "t_name",
    "TaskInstanceId": "task_id",
    "TaskInstanceRef":[
     {
       "url":"url"
       "entrypoint":"entrypoint"
     }
    ] 
    "Status" : "Status"
  }
]
```
Status: 201 (Initilised), 201(Created), 203(Failed), 204 (No Content), 400 (Bad Request)


### GET /deployment/task/{ task_request} 
Get the information about a deployed service
Param:
```json
[
  {
    "TaskInstanceName": "t_name",
    "TaskInstanceId": "task_id",
    "last_knownstatus" : "Status"
  }
]
```
Return:
```json
[
  {
    "TaskInstanceName": "t_name",
    "TaskInstanceId": "task_id",
    "Status" : "status"
    "Feedback" : "feedback"
    "TaskInstanceRef":[
     {
       "url":"url"
       "entrypoint":"entrypoint"
     }
    ] 
    "configuration":[
      {
       "placement":"placement"
       "slicing":"slicing"
      }
    ]
  }
]
```
### PATCH /deployment/task/{task_request } 
Scale up and down of an existing network service
Param:
```json
[
  {
    "TaskInstanceName": "t_name",
    "TaskInstanceId": "task_id",
    "last_knownstatus" : "Status"
    "configuration":[
     {
       "placement":"placement"
       "slicing":"slicing"
     }
    ] 
  }
]
```
Return:
```json
[
  {
    "TaskInstanceName": "t_name",
    "TaskInstanceId": "task_id",
    "Status" : "status"
    "Feedback" : "feedback"
    "TaskInstanceRef":[
     {
       "url":"url"
       "entrypoint":"entrypoint"
     }
    ] 
    "configuration":[
      {
       "placement":"placement"
       "slicing":"slicing"
      }
    ]
  }
]
```
### DELETE /deployment/ns/{task_request } 
Terminate an existing network service
Param:
```json
[
  {
    "TaskInstanceName": "t_name",
    "TaskInstanceId": "task_id",
    "last_knownstatus" : "Status"
  }
]
```
Return:
```json
[
  {
    "TaskInstanceName": "t_name",
    "TaskInstanceId": "task_id",
    "Status" : "Status"
  }
]
```


