# CROP Demonstration

## Network-based SLAM in unstructured real-world network using containerised ROS.
### 1) Offloading SLAM to remote Cloud or local Edge (BUT)
### 2) Quality-aware operations, robot selecting optimal resource based on quality signal. (NEC, BED)

Requirements: ROS 2 Hunble Desktop + docker installed.

For clouds robotics, it is critical that robots have the knowledge of the enviroment in terms of signal quality to properly decide upon offloading of resources based on the tasks criticality and priority.
This knowledge of the enviroment can be achieved using the network application for semantic map developed for CROP.

Using the CROP relay and a Middleware instance, a robot can request a deployment of the signal quality map application:

Use the following parameters to call for the middleware deployment:

Semantic map - MIDDLEWARE_TASK_ID: b96277b-83ff-43ee-88be-aae512c270ff
MIDDLEWARE_ROBOT_ID: 300c719a-1c06-4500-a13a-c2e20592b273

Change accordingly robot middleware IP address: 
MIDDLEWARE_ADDRESS=192.168.50.80:31000

```
docker run --network host --rm -e USE_MIDDLEWARE=true -e MIDDLEWARE_ADDRESS=192.168.50.80:31000 -e MIDDLEWARE_USER=ad20f254-dc3b-406d-9f15-b73ccd47e867 -e MIDDLEWARE_PASSWORD=middleware -e MIDDLEWARE_TASK_ID=8b96277b-83ff-43ee-88be-aae512c270ff -e MIDDLEWARE_ROBOT_ID=300c719a-1c06-4500-a13a-c2e20592b273 but5gera/ros2_relay_client:1.1.0
```

Within the robot, you may run 

```
ros2 topic list
```

You should have /semantic_plc and /current_semantic_pcl topics. In Rviz you can visualice the pointcloud data by adding a pointcloud2 object and resize to 0.5. Use /semantic_pcl topic to see the semantic map during the robot exploration.

The robot can be teleoperated or sent a 2dGoal request. You can simulate the signal quality by chaning the colors ranges by running the following node:

```
ros2 run era_5g_network_signal_mapper_ros2 color_pub
```

After the exploration is completed, a 2dOccupancyGridMap can be saved from the semantic map that will translate low quaility areas (red) to virtual occuapncy cells. In this way, ROS2 Nav2 will treat these objects as real objets in the scenario.

To get this map, from within the robot terminal:

```
ros2 run era_5g_network_signal_mapper_ros2 costmap_translate
```

Additionally, the semantic map pcl2 can be saved as a txt file by using a ROS2 service call.

```
ros2 service call /save_pointcloud std_srvs/Trigger "{}"
```

To load this semantic map at any point, use the ROS2 servive call:

```
ros2 service call /load_and_publish_pointcloud std_srvs/Trigger "{}"
```
The expected output is: 
![Alt Text](https://private-user-images.githubusercontent.com/26432703/270154809-19411486-743c-4fe0-a5e6-3dd4fcd7e37b.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MTAyNDU2ODQsIm5iZiI6MTcxMDI0NTM4NCwicGF0aCI6Ii8yNjQzMjcwMy8yNzAxNTQ4MDktMTk0MTE0ODYtNzQzYy00ZmUwLWE1ZTYtM2RkNGZjZDdlMzdiLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDAzMTIlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQwMzEyVDEyMDk0NFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTU4YTQ2ZjUxOTViZDE2OTAyY2Q4NDZkZTk5MTVmMDZiY2FmNGFiNWFmZTAxN2NhZWI1ZmQwODhmNTNiYmQ0NGYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.xmCQSc1udQIerXX8ptA40ARDnTx1uOd3iM-FC9mRZnk)




### 3) Resilient and robust service with autonomous Edge switch over (BED)
## The demonstration is hybrid:
### 1) Remote demonstration by summit XL connected to AWS Cloud (ROB, BED)
### 2) Onsite demonstration by simulated robot with pre-recorded point cloud data (ROB, BED, BUT)
