# REDIS GRAPH

Todo

## REDIS METADATA_&_QoS

## Policy

```json
{
"Id": "guid",
"Timestamp": "2022-03-11T12:09:43.5122616+00:00",
"IsActive": "True/False",
"Description": "Try to use closest physical machine in the topology",
"PolicyName": "Closest_Machine",
}
```

## DIALOGUES

```json
{
"Id": "guid",
"Question": "Do you have a map?",
"IsSingleAnswer": true,
"Answer": [{"key": "value"}]
}
```

## CONTAINER_IMAGES

```json
{
"Id": "guid",
"Name": "Name",
"Timestamp": "2022-03-11T12:09:43.5122616+00:00",
"Description": "Lorem Ipsum",
"K8SDeployment": "Description of the deployment in the form of the yaml file",
"K8SService": "Description of the service in the form of the yaml file",
}
```

## Task

```json
{
  "Id": "Guid",
  "Timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "TaskId": "Guid",
  "TaskDescription": "Lorem Ipsum",
  "TaskPriority": 1,
  "TaskMaxDuration": 0,
  "TaskStatus": "running",
  "TaskWithMultipleRobots": false,
  "Robot_Id": [ { "Robot_Id": "guid" } ],
  "ActionPlanId": "Guid",
  "QoE": 95,
  "TaskExecutionTime": 120
}
```

## ROBOT

```json
{
    "Id": "Guid",
    "RobotName": "RobotName",
    "Manufacturer": "RobotNik",
    "RobotModel": "Summit-xl",
    "RobotStatus": "Running/withOutBattery",
    "CurrentTaskID": "Task_Number",
    "BatteryStatus": 90,
    "MacAddress": "MacAddress",
    "LocomotionSystem": "Ackerman/differential_Drive",
    "ArticulationAvailable": "false",
    "NumberOfArticulation": 0,
    "ArticulationDof": [],
    "Sensors": ["lidar", "camera", "IMU"],
    "CPU": 90,
    "RAM": 90,
    "VirtualRam": 90,
    "StorageDisk": 90,
    "NumberCores": 3,
    "Timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "Questions": [
        {"map": "present, none"},
        {"Question":"Answer"}
    ]    
  }
```

## EDGE

```json
  {
    "Id": 10,
    "EdgeName": "EdgeName",
    "EdgeStatus": "Running/withOutBattery",
    "EdgeIp": "192.168.1.2",
    "MacAddress": "MacAddress",
    "CPU": 90,
    "RAM": 90,
    "VirtualRam": 90,
    "DiskStorage": 90,
    "NumberOfCores": 3,
    "Timestamp": "2022-03-11T12:09:43.5122616+00:00"
  }
  ```

## CLOUD

```json
 {
    "Id": 10,
    "CloudName": "CloudName",
    "CloudStatus": "Running",
    "CloudIp": "192.168.1.2",
    "Timestamp": "2022-03-11T12:09:43.5122616+00:00"
  }

```

## ACTION

Action model Family hierarchy:

* navigation,
* perception,
* manipulation,
* human-assisted navigation,
* human-assisted perception,
* human-assisted manipulation

```json
{
      "Id": "Guid",
      "ActionFamily": "Manipulation",
      "ActionName": "Lorem ipsum",
      "ActionDescription": "Lorem ipsum",
      "TimeStamped": "2022-03-11T12:09:43.5122616+00:00"     
   } 
 ```

## ACTION SEQUENCE

 ```json
 {
 "ActionPlanId": "Guid",
 "ActionSequence": [
    {
      "Id": "Guid",
      "ActionName": "Slam",
      "ActionStatus": "running",
      "Order": 1,
      "Placement": "Edge",
      "ActionPriority": "1",
      "Services": [
        {
          "InstanceId": "6a97703d-c5b2-40ba-a326-5d4b532ef246",
        }
      ]
    },
    {
      "Id": "Guid",
      "ActionName": "2DNavigation",
      "ActionStatus": "running",
      "Order": 2,
      "Placement": "Cloud",
      "ActionPriority": "1",
      "Services": [
        {
          "InstanceId": "c26402d5-73cd-4a23-ae6d-25d97218b762",
        }
      ]
    }
  ]
    }
```

## INSTANCE

```json
{
  "Id": "guid",
  "InstanceName": "Object detection service",
  "InstanceId": "guid",
  "InstanceType": "Object detection/SLAM",
  "IsReusable": true,
  "DesiredStatus": "created",        
  "InstanceUrl": "https://...../......",
  "InstanceStatus": "Active/Down/Instantiating/Idle/Terminating/Problem",
  "TimeStamped": "2022-03-11T12:09:43.5122616+00:00"  
}
```
