# Resource Planning and allocation
Resource Planning is based on resource allocation and service provisioning. Resource allocation plays a vital role in executing the required tasks. Regardless of allocating the resources locally or globally, there are numerous steps that go into it. 5G-ERA Middleware provides a framework that helps cloud-native resources to be allocated and deployed effortlessly. To complete a particular task, a Robot has to complete some steps, and a Resource Planner helps in executing these tasks smoothly. When the Robot asks middleware for the deployment of a certain task, middleware then asks the Resource Planner for the availability of resources. Depending on the availability of resources whether the task needs to be done locally or globally, resources are allocated respectively. The Orchestrator also plays a crucial component in the Middleware. It delivers the integration between the planning mechanism and the Kubernetes platform that allows for the automated deployment of the Network Applications. The orchestrator is responsible for the state and lifecycle management of the Network Applications and enables their management in the Kubernetes cluster. The orchestrator coordinates the deployment and manages the applications’ lifecycle under the Middleware. When the Orchestrator receives the new plan has been instantiated, it starts orchestrating all the resources needed. Furthermore, the network application developers have set priorities and parameter requirements for cloud and edge onboarding that are discussed below. The signal mapping is through color-coding (green, yellow, and red) which has set range by the network application developers as follows:

```json

"Latency":
  [
    {
      "Expected": "100",
      "Green": " = 100",
      "Yellow": "GREATER than 100 and less than 150", 
      "Red": "Greater than 150"
      "Priority": 1
    }
  ]
,
  "Throughput": 
 [
    {
      "Expected": "150",
      "Green": 150,
      "Yellow": "Greater than 140 and less than 150", 
      "Red": "LESS THAN 140"
      "Priority": 2
    }
  ]
,
"Minimum cores":
[
    {
     "Expected": "4",
     "Green": 4,
     "Yellow": "Greater than 3 and less than 4",
     "Red": "less than 3"
     "Priority": 3
  }
]
,
"Minimum ram":
[
   {
    "Expected": "16",
    "Green": 16,
    "Yellow": "Greater than 12 and less than 16",
    "Red": "less than 12"
    "Priority": 4
   }
]
,
"Disk Storage":
[ 
   {
    "Expected": "20",
    "Green": 20,
    "Yellow": "Greater than 16 and less than 20",
    "Red": "less than 16"
    "Priority": 5
   }
]
,
"Is reusable":
[
  {
   "Expected": "True",
   "Green": "True",
   "Red": "False",
   "Priority": 6
   }
]

"
```

The parameters that are covered here are latency, throughput, minimum cores, minimum RAM, and disk storage, and these parameters could be increased by adding more parameters in future. 
The next part of resource planning is cloud and edge onboarding and the templates are mentioned below respectively; 

## Cloud Template 

```
{
"Latency": integer,
"Throughput": integer,
“locality”: false, #not included for now 
“region”: N/A, 
“Number of Cores”: integer,
“Disk storage”: integer,
“RAM”: integer,  
“signal map”: string,    
 }

```

## Edge Template 

```
{
"Latency": integer,
"Throughput": integer,
“locality”: Boolean, #not included for now 
“Region”: string,
“Number of Cores”: integer,
“ram”: integer,
“disk storage”: integer,
“signal map”: string”
}
```

Furthermore, to understand the resource allocation, some scenarios have been created and color code has also been defined: 

### Scenario 1
<table style="width: 98%; margin-right: calc(2%);">
    <tbody>
        <tr>
            <td style="width: 19.9372%;"><br></td>
            <td style="width: 19.9372%;">Latency</td>
            <td style="width: 19.9686%;">Throughput</td>
            <td style="width: 19.9686%;">Number of Cores</td>
            <td style="width: 20.0000%;">Ram</td>
            <td style="width: 20.0000%;">Disk Storage</td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Cloud</td>
            <td style="width: 19.9372%;">100ms</td>
            <td style="width: 19.9686%;">150mb</td>
            <td style="width: 19.9686%;">20</td>
            <td style="width: 19.9686%;">16gb</td>
            <td style="width: 19.9686%;">20gb</td>
            <td style="width: 19.9686%;"><br>
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Edge</td>
            <td style="width: 19.9372%;">90ms</td>
            <td style="width: 19.9686%;">150mb</td>
             <td style="width: 19.9686%;">12</td>
              <td style="width: 19.9686%;">16gb</td>
               <td style="width: 19.9686%;">20gb</td>
            <td style="width: 19.9686%;">
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
    </tbody>
</table>

In this scenario, when Robot queries the Redis graph on who could provide the requirements for task allocation whether edge or cloud. In this case of all green mapping for edge and cloud, Policy will play a vital role in resource allocation. Policies in the middleware are predefined. Users manage the operational properties of the policies but are restricted in adding and deleting new custom policies. The policies in middleware are available under two different scopes : 
System-scoped -The system-scoped policies are functioning in the specific parts of the whole Middleware and affect the behavior of the concrete functionalities. 
Resource-scoped - The resource-scoped policies are attached to specific resources like Robots or Network applications to alter the way Middleware interacts with them. Multiple policies can be applied to the Network Applications.
For example; Location Selection policies, allow you to customize the location selection process. At this moment, there are two policies that allow altering the location selection process : 
Urllc Slice Location searches for the middleware location that has the slicing mechanism and 
default location selection that will always try to deploy the network applications in the current location. It means the returned location will have the properties of the Middleware and the client is contacting using API calls. 
In this particular case, the nearest placement is utilized as per the policy and the task allocation( ex-object detection netapp) will be deployed in the nearest edge. 

### Operational 

<table style="width: 98%; margin-right: calc(2%);">
    <tbody>
        <tr>
            <td style="width: 19.9372%;"><br></td>
            <td style="width: 19.9372%;">Heartbeat Quality</td>
            <td style="width: 19.9686%;">Signal quality</td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Operational</td>
            <td style="width: 19.9372%;">Good</td>
            <td style="width: 19.9686%;">Good</td>
            <td style="width: 19.9686%;"><br>
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
    </tbody>
</table>

### Priority
* Latency - 1
* Throughput -2 
* Locality -3
* Number of Cores -4
* Ram -5 
* Disk Storage -6

## Colour
* Green – 1
* Yellow – 2
* Red -3 

Formula: score = sum(p*c)

In Scenarion 1, Cloud =Edge 
In scenario 1, since both edge and cloud mapping is green and the operational is also green, the policy (nearest resource edge) will be utilized and the OD (Object Detection) netapp will be deployed in edge. 

### Scenario 2 
<table style="width: 98%; margin-right: calc(2%);">
    <tbody>
        <tr>
            <td style="width: 19.9372%;"><br></td>
            <td style="width: 19.9372%;">Latency</td>
            <td style="width: 19.9686%;">Throughput</td>
            <td style="width: 19.9686%;">Number of Cores</td>
            <td style="width: 20.0000%;">Ram</td>
            <td style="width: 20.0000%;">Disk Storage</td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Cloud</td>
            <td style="width: 19.9372%;">100ms</td>
            <td style="width: 19.9686%;">140mb</td>
            <td style="width: 19.9686%;">4</td>
            <td style="width: 19.9686%;">16gb</td>
            <td style="width: 19.9686%;">20gb</td>
            <td style="width: 19.9686%;"><br>
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
            <td style="width: 19.9372%;">Edge</td>
            <td style="width: 19.9372%;">90ms</td>
            <td style="width: 19.9686%;">140mb</td>
             <td style="width: 19.9686%;">2</td>
              <td style="width: 19.9686%;">10gb</td>
               <td style="width: 19.9686%;">20gb</td>
            <td style="width: 19.9686%;">
                <table style="width: 100%;">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
        <tr>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
            <td style="width: 20.0000%;">
                <table style="width: 98%; margin-right: calc(2%);">
                    <tbody>
                        <tr>
                        </tr>
                    </tbody>
                </table><br>
            </td>
        </tr>
    </tbody>
</table>


Colour: 
* Green – 1
* Yellow – 2
* Red -3 

Formula: score = sum(p*c)

* Cloud = 1+ 4 + 3 + 4 + 5 + 6 = 23
* Edge =  1 + 4 + 3 + 8 + 10 + 6 = 32

Cloud < edge (OD deployed in cloud) 
In this third scenario, when Robot queries the Redis graph on who could provide the requirements for task allocation whether edge or cloud. According to the formula, The task allocation object detection netapp will be deployed in the cloud. 