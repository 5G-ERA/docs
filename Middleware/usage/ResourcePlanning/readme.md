
# Planning & replanning features in the 5G-ERA Middleware:

The 5G-ERA middleware has a planning module for action and resource planning. This allows a robot to ask for a high level task and allow the middleware to provide low level action sequence with required algorithms as containers resembling KNF and VNF. Planning is divided in two main categories depending if the context surrounding the robot is known or not.

## 1) Known Context Planning:

This means that the predefined action sequence stored in redis knowledge graph is adapted specifically for the Task with known context (location, enviroment, robot). This is the classical approach in planning and works like an expert system that restrains uncertainty to be minimal and oriented to failure in the robot algorithms, OS or HW, however, the context outside the robot will not change or is expected not to change. This planning is triggeered when the parameter **BOOL_CONTEXT_KNOWN** is set to true, allowing the middleware to know that unknown situations will not happend as far as the surrounding enviroment is concerned. This is specially true for static robots robots that work in factories and brazing cells. The client strongly believes the outside factors will not change and provides a deterministic action sequence for the redis knowledge graph.

Calling the API planning endpoint for a known context plan will require providing the following data:

#### GET /PLAN/{param}
```json
{
  "RobotId" : "Guid",
  "LockResourceReUse": "false",
  "ReplanActionPlannerLocked": "false"
  "TaskId" : "task_id",
  "TaskDescription": "string",
  "ContextKnown" : "true",
  "Questions": []
}
```

* The RobotId is a unique identifier generated after the [registration](https://github.com/5G-ERA/middleware/blob/main/docs/1_Middleware/3_Architecture/Gateway/ProposedInterface.md)
* LockResourceReUse parameter avoids the middleware from trying to reuse some of the containers from other past deployments with same instances. Recommed to be false by default.
* ReplanActionPlannerLocked: Do not modify action sequence if replan is neccesary, only placement
* TaskId parameter is automatically generated when performed a new [onboarding_task](https://github.com/5G-ERA/middleware/blob/main/docs/1_Middleware/3_Architecture/RedisInterface/ProposedInterface.md). Let's recall that it will contain a predefined action sequence with the **Known Context** perspective.
* Questions: list of questions template including task criticality, priority, danger etc. This will mainly be applied to the resource planner to choose a much power powerful machine to avoid failure.

For more information about this API endpoint, check [API](https://github.com/5G-ERA/middleware/)

### 1.1) Planning endpoint
___
The parameters from the API call are fetch into the system and the actionPlanner Module starts. 

1. The taskId is checked to be registered in the redis graph. (If not present, the task will be rejected as **ContextKnown** is set to true.)

2. A new plan Id is automatically generated for this request along side a task object.

3. For each action in the action sequence retreived from redis, the middleware will check the robot has the sensors and actuators neccesary for the vertical netApps(instance algorithms).

4. The actions will be added to new task object as part of the action sequence.

5. The object will be given to the resource planner. This one will associate the neccesary instances (algorithms) with each action.

6. For each action, a best placement will be found by considering the active resource type policies of the system. The placement attribute of each action will be updated.

7. The Task with all the neccesary data will be given to the orchestrator to save the new plan in redis along side with a timestamp and the robotId that requested the task. 

8. The orchestrator will deploy the containers in dedicated locations. 
9. The orchestrator will update redis graph knowledge model with new relationships between robot and consumed resoruces.
10. The 200 OK respond is given back to the robot with the plan information, plan id and placement chosen as well as other useful data.
___
### 1.2) Replanning endpoint

If the robot fails to execute an action from the action sequence or it wants to get another plan, the replan endpoint will be called. In this case, the robot may ask for a **complete replan** or a **partial replan**. The main difference here is that for partial replan, the middleware will create a new action sequence that may not include the previous succesful actions unless they are quiried again becasuse of the nature of the task. This information, named the Markovian property of an action is established during the [onboarding of task](https://github.com/5G-ERA/middleware/blob/main/docs/1_Middleware/1_Onboarding/Task.md) and a new pre-establehed action sequence.

The basic idea, however, is explained in the figure below. Action 2 depends on action 1, meaning that if action 1 failed and the robot just wants to do action 2, this would not be possible. The middleware would alocate an action sequence with action 1 and 2 in that order. This information must be provided during onboarding of the new task to specify the nature of each of the actions. 

<p align="center">
  <img src="https://github.com/5G-ERA/middleware/blob/main/docs/img/imagen_2022-10-08_183127880.png?raw=true" alt="Middleware architecture"/ width="400" 
     height="250">
</p>

To allow the middleware to do partial replan, when calling the **GET replan endpoint** the parameter **CompleteReplan** must be set to false as in the example below.
#### 1.2.1) Partial replanning 

#### GET /replan/{param}

```json
{
  "RobotId" : "Guid",
  "LockResourceReUse": false,
  "ContextKnown" : true,
  "CompleteReplan" : false
  "TaskId" : "task_id",
  "Questions": []
}
```

___
The parameters from the API call are fetch into the system and the actionPlanner Module starts.

1. The taskId is checked to be registered in the redis graph. (If not present, the task will be rejected as **ContextKnown** is set to true.)

2. A new plan Id is automatically generated for this request along side a task object. **The replan tag of the task is now active** to know that this plan is indeed a replan f type full or partial.

3. Query Redis for old plan and obtain the old action seq.

4.  Check if the flaf **ReplanActionPlannerLocked** is only task planed is set to true. If so, the robot requested to not change anything from action sequence but placement, send new task with old action seq and new action plan id to the resoure planner. Else:

5. If partial or complete replan, find alternative candidate instances (**other algorithms of same family and ros specifications**) and add them to the actions seq. If partial replan, try to only add the actions failed but also consider their Markovian properties.

6. In resource planner now: Check in which of the failed actions action planner has not done some modifications.
7. Find a better placement to the old action for those actions according to established policies.
8. Send new task back to the robot with the updated replan.

Example of call plan to API:
```
http://localhost:5047/Task/Plan?dry=true
```

```json
{
    "RobotId": "62ccb5d8-dc1a-4c57-9068-062bde5c57a7",
    "LockResourceReUse": true,
    "ReplanActionPlannerLocked": true,
    "TaskId":"fbbfd363-923c-414f-ad6c-5b28fa338576",
    "TaskDescription": "Lorem ipsum",
    "ContextKnown": true,
    "Questions": [
    {
      "relations": [
        {
          "initiatesFrom": {
            "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
            "type": "string",
            "name": "string"
          },
          "relationName": "string",
          "relationAttributes": [
            {
              "key": "string",
              "value": "string"
            }
          ],
          "pointsTo": {
            "id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
            "type": "string",
            "name": "streing"
          }
        }
      ],
      "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
      "Question": "string",
      "IsSingleAnswer": true,
      "Answer": [
        {
          "key": "string",
          "value": "string"
        }
      ]
      }
  ]
}
```
___

## 2) Unknown Context Planning:

*Feature to be included during the year 2023-2024*

