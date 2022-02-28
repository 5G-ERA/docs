# Endpoints

## Deployment

### POST /deployment/ns/{name} 
Get client/robot request for task help.
Param: name - Name of the NS
Return: ID of deployed NS as GUID
Status: 201 (Created), 204 (No Content), 400 (Bad Request)

## GET /help request general/ns/ 
Get all the NS request for task help
Param: None
Return: 
```json
[
  {
    "REQUESTID": "REQUEST1",
    "status": "issued/in_process/fail/queued/imposible"
    "Timestamped" : "YY_MM_DD_HH_MM_SS"
  }
]
```
Status: 200 (OK), 404(Not Found)

### GET /help request details/ns/{name} 
Get the NS instances for the specific NS
Param: name - Name of the NS
Return: 
```json
[
  {
    "TASK_ID": "TASK_1",
    "REQUEST_ID": "REQUEST_1",
    "ROBOT_ID": "ROBOT_1",
    "PRIORITY": "MEDIUM",
    "Timestamped" : "YY_MM_DD_HH_MM_SS"
  }
]
```
Status: 200 (OK), 404(Not Found)

### GET /QUESTION/{id} 
Get the NS instance for Q&A by its ID
Param: id - Id of the NS
Return: 
```json
[
  {
    "QUESTION_ID": "QUESTION_1",
    "name": "ASK_FOR_MAP",
    "status": "asked/answered/ignored/dont_know"
    "answer": "String/integer/Boolean"
    "Timestamped" : "YY_MM_DD_HH_MM_SS"
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /OUTPUT/{id} 
Get the NS action planner output by its ID
Param: id - Id of the NS
Return: 
```json
[
  {
    "ACTION_SEQUENCE": "[ACTION_1,ACTION_2,ACTION_3]",
    "ACTION_PLANNER_OUTPUT_ID": "ACTION_PLANNER_OUTPUT_1",
    "status": "queued/fail/in_process/completed/started/idle"
    "Timestamped" : "YY_MM_DD_HH_MM_SS"
  }
]
```

Status: 200 (OK), 404(Not Found)
