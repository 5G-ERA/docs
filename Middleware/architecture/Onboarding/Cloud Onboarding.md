# Cloud Onboarding
For the middleware to plan optimal network application placement, it is important to import the network topology into the system. In this section, we will learn how to create a new Cloud entity in the Redis backend of the Middleware.

Adding a new entry in the Middleware topology allows Middleware to better optimize the resource and task planning to provide the best network capabilities to the Robot. Thanks to this, the Network Application placement can be adjusted to specific needs like low latency network thanks to the closer placement from the Robot or specific `network slice` requirement (upcoming!).

### Step 1: Authentication
The user needs to be registered with the middleware system. For registration, the Postman API platform/Insomnia is required. After registration, a token will be generated which will be used to create a Cloud entity.

### Step 2: Cloud template
```
{
  "Latency": integer,
  "Throughput": integer,
“locality”: false,
“region”: N/A, 
“Number of Cores”: integer,
“Disk storage”: integer,
“RAM”: integer,  
“signal map”: string,    
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



### Step 3 : Configuration of Preferred API 
The properties have to be set as well as part of the configuration of the preferred REST API client like Postman/Insomnia.
•	The IP address of the Middleware
•	Path of a request /data/cloud
•	Request method is set to POST
•	Content-Type header value set to application/json 

### Step 4: Importing the cloud definition 
Before sending a POST request ensure that the token has been added from step 1.1.
After providing the correct token, the request is ready to be executed. The Edge should be accepted and a new ID should be given to the Middleware. 
Cloud Onboarding allows easy integration of the Middleware with other running instances.

