# External Interface

## Registration

### POST /Register
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


## Authentication

### POST /Login
Param:
```json 
{
   "Id":"3cace17e-a091-4f91-ad98-7d0c371b3a25",
   "Password":"password"
}
```
Return:
```json
{
     "token":"JWTBearerToken",
     "expirationDate": "2022-05-09T22:35:56.8472623Z"
}
```

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

## Get Topology for possible placement
### GET /topology/ 
Return list of sites available in the system
Param: none

Return:
```json
[
  {
    "number_of_sites" : "3"
    "site_names": ["Edge1","Edge2","Cloud1"]
    "site_ids":["1", "2", "3"]
    "site_type":["Edge","Edge","Cloud"]
  }
]
```

Return information of a specific site
### GET / topology /site/{site_id} 

Param: site_id

Return:
```json
[
  {
    "CPU" : "status"
    "RAM" : "status"
    "eMBB" : "status"
    "URLLC" : "status"
    "mMTC" : "status"
     }
]
```
Status: available_idle, available_busy, impossible 

