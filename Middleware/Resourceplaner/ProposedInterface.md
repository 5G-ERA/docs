# Endpoints

## Deployment


### GET /ACTION_PLAN/{name} 
Get the NS instances for the specific help request. Asking: RESOURCEPLANNER ANSWER: ACTIONPLANNER.
(Assumptions: robot knows the task_id for a high level action -go to kitchen-).
OUTPUT; ACTION_PLANNER_OUTPUT

```json
[
  {
    "OUTPUT_ID": "OUTPUT_ID",
  }
]
```


Return: 
```json
[
  {
 "ACTION_SEQUENCE": "[ACTION_NUMBER [PRIORITY]]",
 "PRIORITY": "HIGH/MEDIUM/LOW"
  }
]
```


### GET /POLICY/{id} 
Get the NS instance for Q&A by its ID. Asking: RESOURCE PLANNER ANSWER: SEMANTIC DB REDIS.
Param: id - Id of the NS

```json
[
  {
    "RobotId": "NSid",
    "QUESTION_ID": "QUESTION_NUMBER",
  }
]
```

Return: 
```json
[
  {
    "QUESTION_ID": "QUESTION_NUMBER",
    "name": "NAME",
    "status": "asked/answered/dont_know"
    "answer": "String/integer/Boolean"
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /QUESTION/{id} 
Get the NS instance for Q&A by its ID. Asking: 5G ERA ANSWER: ROBOT.
Param: id - Id of the NS

```json
[
  {
    "RobotId": "NSid",
    "QUESTION_ID": "QUESTION_NUMBER",
  }
]
```

Return: 
```json
[
  {
    "QUESTION_ID": "QUESTION_NUMBER",
    "name": "NAME",
    "status": "asked/answered/dont_know"
    "answer": "String/integer/Boolean"
  }
]
```

Status: 200 (OK), 404(Not Found)


### GET /STATEMACHINE_OUTPUT/{id} 
Get the NS action planner output by its ID. Asking: RESOURCE PLANNER ANSWER: ACTION_PLANNER.
Param: id - Id of the NS
Return: 
```json
[
  {
    "ACTION_SEQUENCE": ['ACTION_NUMBER'],
    "OUTPUT_ID": "OUTPUT_NUMBER",
    "status": "queued/fail/in_process/completed/started/idle"
  }
]
```

Status: 200 (OK), 404(Not Found)
