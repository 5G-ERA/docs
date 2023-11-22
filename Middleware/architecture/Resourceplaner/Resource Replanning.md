# Resource Replanning and possible scenarios of Replanning 

Task planner asks resource planner for resources to complete the replanning of task, assuming ROBOT knows Task-Id and the preselected policy.


* Step 1: Administrator get the preselected policy to the robot_1

* Step 2: Get action sequence (Action_1)

* Step 3: Resource planner will iterate through the action sequence
→ Relation model for every relation in images (owns, provides, extends)
→ Mapping all relations between the entities 
* Step 4: List all the relations with images for the action

* Step 5: Robot_1 has the same network connections with Edge_1 and Edge_2 where the middleware is installed. 

* Step 6: Mapping of the instance model 

* Step 7: Middleware informs Robot of the plan 

* Step 8: Redis Graph will be updated with “owns” relationship between Robot_1 and Task_1

* Step 9: Robot_1 provides Task_id to the Middleware 

* Step 10: Middleware infers the action_sequence and placement 

* Step 11: Middleware informs Robot of the plan 

* Step 12: Redis Query will give the info to the Orchestrator “ Resource that are free to perform task Edge_1” 

* Step 13: Actions needed to be instantiated like SLAM, OBJECT DETECTION for the specified tasks. (GET the images that needs to be deployed )

* Step 14: Endpoints of the replanning will be triggered 

* Step 15: Middleware tries to find the another Netapp from the same Netapp family where the netapp is deployed 

* Step 16: Middleware changes the placement (Edge_1) and putting the netapp in more powerful machine (Edge_2) that complies with current policy   based on a)latency b)Remaining Computation c)bandwidth

* Step 17: Assigning new images to that action that is Netapp Object Detection

* Step 18: Orchestrator will pass this information to the Task Planner 

* Step 19: Return Task model with updated images 

```
Possiblities of the Replanning Scenario: 

1. Robot asks for Replan before completing the Task_1 (Robot failed the task_1)

2.  Robot failed Task_1 and the Middleware provides a replan (Partially failed)

3. Middleware will proactively suggest a replan (In this possibility, Robot_1 can accept of reject the replan)
```


## Planning & replanning features in the 5G-ERA Middleware:

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
