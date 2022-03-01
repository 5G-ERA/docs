# Endpoints

## Deployment


### GET /PLAN/{param} 


Asking: ROBOT ANSWER: 5G-ERA.
(Assumptions: robot knows the task_id for a high level action -go to kitchen-).

```json
{
  "RobotId": "guid",
  "TaskId" : "task_id",
  "RobotInfo": {
    "map": "present, none"
    
  }
}
```

Return: 
```json
{
  "TaskId": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        "ServiceType: "Object detecction/SLAM"   
        "ServiceUrl": "https://...../......",
        "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
      ]
   } 
  ]
}
```
Status: 200 (OK), 404(Not Found)

### GET /replan/{param}

Param:
```json
{
  "TaskId": "guid"
  "ActionPlanId": "guid",
  "ActionSequence": [
    {
      "ActionId": 2,
      "Status": "Done/In progress/Failed/Unable to execute",
      "Timestamp": "dd/MM/yyyy HH24:ss.mmm"
    }
  ],
  
}
```
Return: 
```json
{
  "TaskId": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "ServiceUrl": "https://...../......",
      "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
   } 
  ]
}
```
Replan 
