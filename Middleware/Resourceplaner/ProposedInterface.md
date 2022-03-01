# Endpoints

## Deployment


### GET /Resource/{name} 
Asking: ACTION PLANNER  ANSWER: RESOURCE PLANNER.
(Assumptions: robot knows the task_id for a high level action -go to kitchen-).
OUTPUT; ACTION_PLANNER_OUTPUT

```json
{  
  "ActionPlanId": "guid",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "ActionPriority": 1/2/3,      
   } 
  ]
}
```


Return: 
```json
{
  "TaskId": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": guid,
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,      
      "ActionPriority": 1/2/3,
      "Placement: "EDGE/CLOUD",
      "ServiceId/Image name": "Object detection service"
   } 
  ]
}

```


### GET /POLICY/Current

**TODO**: Move to Redis API

Asking: RESOURCE PLANNER ANSWER: SEMANTIC DB REDIS.

Return: 
```json
[
  {
    "PolicyId": 10,
    "PolicyDescription": "lorem ipsum"    
  }
]
```

Status: 200 (OK), 404(Not Found)
### GET /POLICY/All

**TODO**: Move to Redis API

Asking: RESOURCE PLANNER ANSWER: SEMANTIC DB REDIS.

Return: 
```json
[
  {
    "PolicyId": 10,
    "PolicyDescription": "lorem ipsum",
    "IsActive": "true/false",
    "Timestamp": "dd/MM/yyyy"
  }
]
```

Status: 200 (OK), 404(Not Found)

### GET /GRAPH_TOPOLOGY/{id} 

**TODO**: Move to Redis API

RESOURCE PLANNER ANSWER: REDIS GRAPH
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

