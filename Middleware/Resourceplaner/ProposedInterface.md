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
      "Id": 2,
      "Order": 0,
      "ActionPriority": "1/2/3",      
   } 
  ]
}
```

Return:

```json
{
  "Id": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "Id": 2,
      "Order": 0,      
      "ActionPriority": "1/2/3",
      "Placement": "EDGE/CLOUD",
      "ServiceId/Image name": "Object detection service"
   } 
  ]
}

```

### GET /GRAPH_TOPOLOGY/{id}

**TODO**: Move to Redis API
Function to ger a GRAPH.

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
