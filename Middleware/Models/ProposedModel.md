# REDIS GRAPH

Index db 1




# REDIS METADATA_&_QoS 

Index 2

## Policy

```json
{
"Policy_Id": "guid",
"Timestamp": "dd/mm/yyyy",
"IsActive": "True/False",
"Description": "Try to use closest physical machine in the topolgy",
"PolicyName": "Closest_Machine",
}
```

## DIALOGUES

```json
{
"Question_Id": "guid",
"Question": "Do you have a map?",
"IsSingleAnswer": true,
"Answer": [{"key": "value"}]
}
```

## CONTAINER_IMAGES

```json
{
"ImageId": "guid",
"Name": "Name",
"Timestamp": "dd/mm/yyyy",
"Description": "Lorem Ipsum"

}
```

## Task

```json
{
"Request_ID": "guid",
"Timestamp": "yyyy/mm/dd hh:mm.ss",
"TaskId": "TASK_NUMBER",
"TaskDescription": "Lorem Ipsum",
"TaskPriority": "HIGH/MEDIUM/LOW",
"TaskMaxDuration": 0,
"Robot_Id":  "Robot_1",
"ActionPlanId": "guid",
"ActionSequence": [
  {
    "ActionId": 2,
    "ActionName": "Slam",
    "Order": 0,
    "Placement": "EDGE/CLOUD",
    "ActionPriority": "1/2/3",
    "Services": [
      {
        "ServiceId/Image name": "Object detection service",
        "ServiceInstanceId": "guid",
        "ServiceType": "Object detection/SLAM",
        "IsReusable": true,
        "DesiredStatus": "created",        
        "ServiceUrl": "https://...../......",
        "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
      }
    ]      
  } 
]
}
```

## ROBOT

```json
{
    "RobotID": "Robot_1",
    "Manufacturer": "RobotNik",
    "RobotModel": "Summit-xl"
    "RobotStatus": "Running/withOutBattery",
    "CurrentTaskID": "Task_Number",
    "BatteryStatus": 90,
    "MacAddress": "MacAddress",
    "LocomotionSystem": "Ackerman/differential_Drive",
    "ArticulationAvailable": "false",
    "NumberOfArticulation": 0,
    "ArticulationDof: []",
    "Sensors": ["lidar", "camera", "IMU"],
    "CPU": 90,
    "RAM": 90,
    "VirtualRam": 90,
    "StorageDisk": 90,
    "NumberCores": 3,
    "Questions": [
        {"map": "present, none"},
        {"Question":"Answer"}
    ]
    
  }
```

## EDGE

```json
  {
    "EdgeID": 10,
    "EdgeStatus": "Running/withOutBattery",
    "EdgeIp": "192.168.1.2",
    "MacAddress": "MacAddress",
    "CPU": 90,
    "RAM": 90,
    "VirtualRam": 90,
    "DiskStorage": 90,
    "NumberOfCores": 3
  }
  ```

## CLOUD 

```json
 {
    "CloudID": 10,
    "CloudStatus": "Running",
    "CloudIp": "192.168.1.2"    
  }

```
## ACTION
```json
{
      "ActionId": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
          "ServiceDataModel":
        }
      ]      
   } 
 ```

## Service data model:
```json
{
  "ServiceId": "guid",
  "ImageName": "Object detection service",
  "ServiceInstanceId": "guid",
  "ServiceType": "Object detection/SLAM",
  "IsReusable": true,
  "DesiredStatus": "created",        
  "ServiceUrl": "https://...../......",
  "ServiceStatus": "Active/Down/Instanciating/Idle/Terminating"
}
```

##Instance:
```json
{
  "InstanceID": 1,
  "InstanceStatus": "Instanciated/failed",
  "Timestamp": "DD/MM/YYYY"
  }



