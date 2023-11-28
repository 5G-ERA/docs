## Onboarding your 5G-ERA Middleware system:

By default, the middleware will have to be populated with your enviroment, robots and custom tasks for your experiments to be conducted. The topology for your cloud
system needs to be created for the resource planner to accurately plan best placement of the NetApp's and actions. The action planner would require the robot ROS specifications
as well as the locomotion system, sensors and other parameters. Therefore, before using the middleware, the onboarding process needs to be finished.

1) First register yourself as a new [user](User.md) or use a user given by your 5G-ERA middleware system administrator.
2) [Login](User.md) to get a temporal token to be used in the upcoming onboarding requests.
3) Check that your robot is in the [list](robot_examples/) of already onboarded robots to the middleware. If so, you dont have to prepare the onboarding file for your robot from scrath, but rather modify the one already available.
4) Onboard your robot following the instructions [here](Robot.md).
5) Onboard your network topologies using [edges](Edge.md) and [clouds](Cloud.md).
