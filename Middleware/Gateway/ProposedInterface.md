# Endpoints

## Authentication
TBD
### /authentication/
TBD

## Deployment

### POST /deployment/ns/{ns_request} 
Initiating a new network service in the OSM
Param:
```json
[
 {
   "name": "name",
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
    "InstanceName": "name",
    "InstanceId": "id",
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


### GET /deployment/ns/{id} 
Get the information about a deployed service
Param:
```json
[
  {
    "InstanceId": "id",
    "InstanceName": "name",
    "last_knownstatus" : "Status"
  }
]
```
Return:
```json
[
  {
    "InstanceName": "name",
    "InstanceId": "id",
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
### PATCH /deployment/ns/{id} 
Scale up and down of an existing network service
Param:
```json
[
  {
    "InstanceId": "id",
    "InstanceName": "name",
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
    "InstanceName": "name",
    "InstanceId": "id",
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
### DELETE /deployment/ns/{id} 
Terminate an existing network service
Param:
```json
[
  {
    "InstanceId": "id",
    "InstanceName": "name",
    "last_knownstatus" : "Status"
  }
]
```
Return:
```json
[
  {
    "InstanceName": "name",
    "InstanceId": "id",
    "Status" : "Status"
  }
]
```
