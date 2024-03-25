## Deployment of Network Application to the Middleware 

In this section, we will learn how to deploy a Network Application to the Middleware system. Essentially, this is the execution of a robotics tasks that has been previously onboarded. So, for the deployment of a network application, the taskid containing the app to be deployed is neccesary. Follow the tutorial [here](https://github.com/5G-ERA/middleware/blob/main/docs/User/Onboarding/Network%20Application%20Onboarding.md) for onboarding of a `Task` if you havent done so.

## Step 1:  Configuration of the preferred REST API Client

As part of the configuration of the preferred REST API client like `Postman` or `Insomnia` the following properties have to be set for the endpoint to call a task execution.

* The IP address of the Middleware 
* Path of a request `task/plan/dryRun=false`
* Request method is set to `POST`
* `Content-Type` header value set to `application/json`

To get the ip of the middleware, you can check it by using the k8s command: 

```
watch -c kubectl get all -n middleware
```

![image](../User/Onboarding/img/deployed_middleware.png)

The headers of the http request should look like this:

![image](../User/Onboarding/img/Netapp%20Header.png)

## Step 2: Task deployment containing Network Application

The full NetApp template looks like this:

```
{
  "TaskId": "7d93728a-4a4c-4dae-8245-16b86f85b246",
  "RobotId": "ed7fbaf5-ac11-468c-9f12-7bf68d8a0a13",
  "LockResourceReUse": true,
  "ReplanActionPlannerLocked": true  
}
```
The `TaskId` property has to be adjusted with the unique identifier of an imported `Task`. Use the identifier of a `Task` that you have imported.

Leave both `LockResourceReUse` and `ReplanActionPlannerLocked` properties as is. The first one makes sure the container cannot be reused by other robots. `ReplanActionPlannerLocked` will make sure to keep the semantic action sequence as is, therefore the Network Application will be deployed as specified in the task definition.

The endpoint has an optional parameter named `dryRun` with the default value `false`. When set to `true` it will request the task definition without deploying it. It can be used to make sure that the `Task` definition has been imported correctly.

Before using this endpoint, make sure you are fist logged in and have and access token. Visit registration [tutorial](../User/Onboarding/User.md) if you dont have a user.

For attaching the token when making a call to one of the endpoints of the Middleware, the user has to do the following.
* Provide the path for to the endpoint
* Select the `Authorization` section
* Choose `Bearer Token` from the dropdown menu
* Paste the `token` in the token section and make the call by clicking `Send` button

Once you have successfully requested a `Task` deployment, in k8s you should be able to see the services and deployed pods of the action sequence of the `Task`.  

![image](../User/Onboarding/img/netApp_deployed.png)
