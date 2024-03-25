# Edge Onboarding

For the middleware to plan optimal network application placement, it is important to import the network topology into the system. In this section, we will learn how to create a new Edge entity in the Redis backend of the Middleware.

Adding a new entry in the Middleware topology allows Middleware to better optimize the resource and task planning to provide the best network capabilities to the Robot. Thanks to this, the Network Application placement can be adjusted to specific needs like low latency network thanks to the closer placement from the Robot or specific `network slice` requirement (upcoming!).

### Step 1:	Authentication
As we mentioned above, adding a new cloud entity allows an easy integration of the Middleware. Similarly, Adding a new Edge entity in the Middleware topology allows the Middleware to better optimize the resource and task planning to provide the best network capabilities to the Robot. Network Application placement can be adjusted to specific resource management like low-latency network and slicing requirements.

### Step 2: Edge template 
```
{
  "Latency": integer,
  "Throughput": integer,
“locality”: Boolean,
“Region”: string,
“Number of Cores”: integer,
“ram”: integer,
“disk storage”: integer,
“signal map”: string”
}
```

•	Latency - Latency is the time it takes for data to pass from one point on a network to another. 
•	Throughput - 
•	Locality – local/ global
•	Region-  the region of the 
•	Number of Cores - the number of the cores the machine consists of
•	Disk Storage - the amount of storage available expressed in GB
•	RAM - the amount of memory the machine has at its disposal, expressed in GB
•	Signal map -  mapping of signal quality


### Step 3:Configuration of Preferred API 
As part of the configuration of the preferred REST API client like (Postman/Insomnia) the following properties have to be set.
 
·        The IP address of the Middleware
·        Path of a request /data/edge
·        Request method is set to POST
·        Content-Type header value set to application/json

### Step 4: Importing the edge definition 
Before sending a POST request ensure that you have added the token obtained in Step 2.1. After providing the correct token, execute the request. The Edge should be accepted and a new ID should be given by the Middleware.
Onboarding of a new Edge allows for the easy integration of the Middleware with other running instances. Middleware is designed for multiple edge and cloud to cooperate together, giving an easy resource integration for cloud-based robotics.
Middleware needs to be acquainted with your environment, Robots, and custom tasks to conduct experiments. The topology for the cloud system needs to be created for the resource planner to accurately plan the best placement of the network applications and actions. The resource planner will need the required parameters and therefore the above onboarding process needs to be finished before using the Middleware. Furthermore, to have more clear ideas, visit the 5g-era middleware page.
