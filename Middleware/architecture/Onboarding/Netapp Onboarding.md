# Network Application Onboarding


This document describes the process of onboarding custom Network Applications to the 5G-ERA Middleware. Network Applications are custom-made containerized applications that are designed to run in a cloud environment.

## Prerequisites

To start the process of onboarding the NetWork Applications the 5G-ERA Middleware needs to be deployed and accessible through the public or private IP address. 

On top of this, the user has to be logged in to access the features of Middleware's API. 

---

## Purpose

The onboarding of the Network Application extends the functionality of the Middleware by allowing it to perform more sophisticated actions for the Robotic vertical applications. 

## Onboarding Template

To onboard the NetWork application the Middleware API has to be called. The REST API has an endpoint at path `data/task/importTask`. For this, the endpoint takes a specific complex parameter in the body.

![import task endpoint](img/ImportTaskEndpointPostman.png)

```json
{
    "Id": "889fc9ac-f803-45ed-a6d4-8df33d4128a3",
    "Name": "Nginx NetWork Application",
    "TaskPriority": 1,
    "ActionSequence": [
        {
            "Id": "765c82db-7a63-45c6-9041-57634424dd23",
            "Name": "Nginx",
            "Order": 1,
            "ActionPriority": "Normal",
            "ActionStatus": "None",
            "MinimumRam": 0,
            "MinimumNumCores": 0,
            "Services": [
                {
                    "Id": "e70c0e00-bda2-46a2-b338-f151c5a9731b",
                    "Name": "Nginx webinar",
                    "IsReusable": false,
                    "MinimumRam":512,
					"MinimumNumCores": 4,
                    "ContainerImage": {
                        "Id": "15889ea9-0909-4b47-8c1a-e6f7322291c9",
                        "Name": "Nginx webinar",
                        "Description": "The example network application onboarding for the webinar",
                        "K8SDeployment": "apiVersion: apps/v1\nkind: Deployment\nmetadata:\n  name: nginx-deployment\nspec:\n  selector:\n    matchLabels:\n      app: nginx\n  replicas: 2\n  template:\n    metadata:\n      labels:\n        app: nginx\n    spec:\n      containers:\n        - name: nginx\n          resources: {}\n          image: nginx\n          ports:\n            - containerPort: 80\n",
                        "K8SService": "apiVersion: v1\nkind: Service\nmetadata:\n  name: nginx-service\nspec:\n  selector:\n    app: nginx\n  ports:\n    - port: 80\n      targetPort: 80\n  type: LoadBalancer\n"
                    }
                }
            ]
        }
    ],
    "Tags": [
        "webinar",
        "5G-ERA",
        "Web"
    ]
}
```

The template has the following structure:

* Task definition
* Action Sequence:
    * Action definition 
    * Services:
        * Configuration of the instance
        * Container Image:
            * Deployment configuration of the specific instance

The template takes as an argument the following properties:

### Task

The `Task` is a high-level definition of a sequence of actions that need to be conducted by the robot to achieve a desired outcome. 

The `Task` has the following structure:

* Id - a unique identifier of a task in GUID format - has to be provided by the user
* Name - the name that will allow easy identification of a task
* Task Priority - the priority of which the task has. Available values are:
    * 1 - Low,
    * 2 - Normal,
    * 3 - High,
    * 4 - Critical
* Action Sequence - contains a list of `Action` definitions used in the task. Each of them can represent a step in the plan that the task is built upon


### Action 

`Action` is an element of the `ActionSequence` that represents a single step during the performance of a `Task`. The `Action` can consist of a simple action that needs only a single Network Application to complete it or a complex set of Network Applications that need to cooperate to achieve it.

The `Action` takes the following structure:

* Id - a unique identifier of action in GUID format - has to be provided by the user
* Name - the name that will allow easy identification of an action
* Order - an order in which the actions will be conducted in the `Task`
* Action Priority - the priority the task has, it allows for the identification of a step that is crucial for the task completion. It can take the following values:
    * 1 - Low,
    * 2 - Normal,
    * 3 - High,
    * 4 - Critical
* Action Status - represents the combined status of all the services within an action
* Minimum Ram - the number of RAM required to run specific action expressed in GB
* Minimum Num Cores - the number of cores that are required to run the application successfully 
* Services - a list of `Instance` definitions needed to complete the specified `Action`

### Instance

An instance is a definition of a Network Application used to perform an `Action`

The `Instance` takes the following parameters:

* Id - a unique identifier of an `Instance` in GUID format - has to be provided by the user
* Name - the name that will allow easy identification of an `Instance`
* Is reusable - can the instance be used by multiple vertical applications
* Minimum Ram - the number of RAM required to run specific instance expressed in GB
* Minimum Num Cores - the number of cores that are required to run the application successfully 
* ContainerImage - a complex property used to define the deployment procedure for the `Instance`

## ContainerImage

The `ContainerImage` is the property that allows specifying the exact configuration of how the `Instance` will be deployed in a Kubernetes cluster.

The `ContainerImage` consists of the following properties:

* Id - a unique identifier of the `ContainerImage` in GUID format - has to be provided by the user 
* Name - a name of a container image used in the deployment
* Description - a description of what the `ContainerImage` is responsible for
* K8SDeployment - a deployment configuration in the `yaml` format parsed to a single-line string
* K8SService - a service configuration that showcases how the service is exposed within or outside of a cluster. The configuration is in a `yaml` format parsed to a single-line string. It is optional as the deployment does not need to be exposed within the cluster.

To correctly and effortlessly convert the `yaml` file into the desired format you can use the [flatten_yaml_to_json_string.py](../../../util/flatten_yaml_to_json_string.py) utility script.

## Configuration of the preferred REST API client

As part of the configuration of the preferred REST API client like `Postman` or `Insomnia` the following properties have to be set.

* The IP address of the Middleware
* Path of a request `/data/task/importTask`
* The request method is set to `POST`
* `Content-Type` header value set to `application/json`

For attaching the token when making a call to one of the endpoints of the Middleware, the user has to do the following.
* Provide the path to the endpoint
* Select the `Authorization` section
* Choose `Bearer Token` from the dropdown menu
* Paste the `token` in the token section and make the call by clicking `Send` button

Then make a call using the **Send** button.

![import task endpoint](img/ImportTaskSendPostman.png)

The returned response with the status code `200` means that the operation has succeeded. Now the `Task` and the Network Application can be deployed through the Middleware.


## Conclusion

This document has presented the way how to import a new Task definition into the Middleware so it can be used by the vertical applications to support their functionality with the cloud processing powers.
