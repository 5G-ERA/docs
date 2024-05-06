# Switchover

5G-ERA Middleware presents the end-user capabilities to dynamically change the location of the Network Applications deployments and the 5G Slice selection. 
The change in the resources is named Switchover. 

## Edge Switchover
Edge Switchover is the process of the on-demand change in the deployment location of the Network Application. When the network issues occur or the Edge the NetApp is currently running on is becoming overloaded, robot or end-user can request redeployment of the resource in a new server. 

### Prerequisites
There are a few requirements that must be met to enable Edge Switchover capabilities:
* There must be at least 2 Middlewares connected to the same infrastructure
* Robot must be executing the ActionPlan

### How it works

When the Edge Switchover is requested, the Middleware associates the `ActionPlan` being executed by the robot with the `Action` and a server that is running on. After recieving the request and associating the resources that will be moved under the Switchover procedure, Middleware sends the request to the second Middleware instance to deploy the requested Network Application in the new server. After the request is successfully received, Middleware starts the process of deleting the Network Application from the origin server. If the Network Application is used by multiple Robots / Users, it will not be deleted, only deployed in a new server. The Robot requesting the Switchover will be notified about the new address to which it should connect to.

After the Switchover process is completed, the Robot is informed in the `ActionPlan` that the address of the Network Application has changed.
