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
    "REDIS_QUERY": "POLICY",
  }
]
```

Return: 
```json
[
  {
    "POLICY": "[POLICY_NUMBER(TRUE/FALSE), POLICY_NUMBER(TRUE/FALSE), POLICY_NUMBER(TRUE/FALSE)]",
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /GRAPH_TOPOLOGY/{id} 
Get the NS instance for Q&A by its ID. RESOURCE PLANNER ANSWER: REDIS GRAPH
Param: id - Id of the NS

```json
[
  {
    "REDIS_GRAPH_QUERY": "GRAPH",
  }
]
```

Return: 
```json
[
  {
    "UPDATED_GRAPH": "GRAPH",
  }
]
```

Status: 200 (OK), 404(Not Found)


### GET /STATEMACHINE_OUTPUT/{id} 
Get the NS action planner output by its ID. Asking: OSM ANSWER: RESOURCE_PLANNER.
Param: id - Id of the NS
Return: 
```json
[
  {
    "RESOURCE_OUTPUT": [['ACTION_NUMBER', 'EDGE/CLOUD','PRIORITY', 'IMAGE'], ['ACTION_NUMBER', 'EDGE/CLOUD','PRIORITY', 'IMAGE'] , ['ACTION_NUMBER', 'EDGE/CLOUD','PRIORITY', 'IMAGE']]
    "REOURCE_OUTPUT_ID": "OUTPUT_NUMBER",
    "status": "queued/fail/in_process/completed/started/idle"
  }
]
```

Status: 200 (OK), 404(Not Found)
