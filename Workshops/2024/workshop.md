# Final demonstration planning

Robot operating autonomously and asking Middleware for plan and conducting:
* Resource Planning using the available locations
* Deployment of the NetApps
* Robot performing Slice Switchover
* Robot utilising NetApp through Middleware
* Robot performing one of or multiple:
	* Train detection
	* Collision avoidance
	* Human Sensing
	* SLAM? - depending on the throughput available
* Robot sending heartbeat to Central API and Middleware
* NetApp quality is bad Edge Switchover requested
* Robot detects the change in the address of the service and connects to the new one
* When the robot is connected, the switchover is completed
* Robot achieves the desired goal (finds a person, completes the map depending on the NetApp)

## Additionally, to be included: 

* Data persistence:
	* Event sourcing 
	* Required by stateful NetApps
	* LLM?
* Utilise semantic signal map for switching between Wi-Fi and 5G?
	* Utilise 2 NIC to perform a switch 
	* Possible reduction in quality and frequency of the send data in the yellow zone
	* In the red zone, the robot switches to a different network communication
	* How to predict the best possible action for the robot?
	
## Task List

**BED**:

-	Resource planning in Middleware needs to be finished and must accept minimum and optimal values (colour code) for the requirement parameters.
-	Define parameters for the connection status to be estimated in the resource planning.
-	Signal quality needs to be included in the robot heartbeat.
-	Architect the state machine to manage the changes in the state of the robot network connectivity.
-	Decision service to change the used network communication method (from Wi-Fi to 5G) should be placed on the robot.
 
**BUT**:

-	Explain how to use online and offline maps/rosbag for frontier explorer with a cartographer.
-	Suggesting range of the QoS for object detection service (the reference network application) 
-	Proposing a protocol between a reference network application and the middleware on adjusting the behaviour of the object detection service in a poorly connected area
 
**Joint**: 

-	Architect robot services and modify the relay client to read heartbeat data and send them to the Middleware. 
-	Architect the solution to read available locations from the CentralApi and initiate a connection to the location with the best connection parameters.
