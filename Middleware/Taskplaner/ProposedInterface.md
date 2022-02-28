# Endpoints

## Deployment


### GET /HELP/ns/{name} 
Get the NS instances for the specific help request. Asking: 5G ERA ANSWER: ROBOT.
(Assumptions: robot knows the task_id for a high level action -go to kitchen-).

```json
[
  {
    "RobotId": "NSid",
  }
]
```

Param: name - Name of the NS
Return: 
```json
[
  {
    "TASK_ID": "TASK_NUMBER",
    "PRIORITY": "HIGH/MEDIUM/LOW",
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
