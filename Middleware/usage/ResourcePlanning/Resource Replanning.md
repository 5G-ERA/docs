# Resource Replanning and possible scenarios of Replanning 

Task planner asks resource planner for resources to complete the replanning of task, assuming ROBOT knows Task-Id and the preselected policy.


* Step 1: Administrator get the preselected policy to the robot_1

* Step 2: Get action sequence (Action_1)

* Step 3: Resource planner will iterate through the action sequence
→ Relation model for every relation in images (owns, provides, extends)
→ Mapping all relations between the entities 
* Step 4: List all the relations with images for the action

* Step 5: Robot_1 has the same network connections with Edge_1 and Edge_2 where the middleware is installed. 

* Step 6: Mopping of the instance model 

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
