# 5G-ERA Hackathon 2023

The repository contains support files for the 5G-ERA 2023 Hackathon. The _era_5g_network_application_template_ folder contains the template for the network applications with basic endpoints and common stuff. The obstacle avoidance is a ROS2 package, which reads data from the 2D laser scanner and drives the robot based on the data. If there is no object in front of the robot, the robot drives forward. If there is some object, the robot will start rotating, until there is no obstacle in front of it. The client.py is an example of the usage of the client python library, which will connect to the network application and start sending the data from the laser_data.txt file and print the results. The client_middleware.py is similar example, but it will also take care of deploying the network application using the middleware. The Dockerfile could be used to create a docker image with the newly created network application.

## Creating a Network Application for obstacle detection with laser

In this session, you will create a new network application to receive the LaserScan data and identify the nearest object to the sensor. 

1) (optional) Install some IDE
    1) e.g., vscode: `snap install code --classic`
2) Clone the Hackathon repository
    1) `git clone https://github.com/5G-ERA/Hackathon2023.git`
1) Install the requirements for the era_5g_network_application_template package.
    1) `sudo apt install python3-pip`
    1) `pip install -r requirement.txt`
    1) `pip install era-5g-client`
3) Rename the era_5g_network_application_template package to some unique name
    1) Name of the package folder
    1) Name of the module folder (era_5g_network_application_template/era_5g_network_application_template)
    1) “name” in the pyproject.toml
    1) “name” in the setup.py
    1) “packages” in setup.py
1) Edit the interface.py file
    1) In the register method, read the scan_width parameter from the args dictionary (see client.py, line 21).
    1) Create a subclass of the TaskHandlerInternalQ (https://github.com/5G-ERA/era-5g-interface/blob/main/era_5g_interface/task_handler_internal_q.py) so it can hold the scan_width parameter value and use it in the register method.
    1) Edit the json_callback_http method
        1) Copy the “core algorithm” from the obstacle_avoidance ros2 package
        2) Get the required data from the “data” variable - the exact format can be seen in the laser_data.txt file
        3) Get the scan_width parameter from the "task" variable - the "task" variable holds the TaskHandler you have created in previous steps and it contains information for the currently connected client / robot.
        4) Using the flask_socketio.send send the results 
1) Run the network application and use the client.py script to test the network application
    1) `python3 interface.py`
    1) `python3 client.py`
1) Create docker image
    1) Edit the Dockerfile - update the name of the package (five times in the Dockerfile)
    1) `sudo docker build . -t 5ghackathon/laser_scan_unique:1.0.0`
        1) The -t parameter specifies the tag (name) of the docker image. The first part, 5ghackathon/, is mandatory; anything after the slash is up to each participant. Each participant must have the unique name of the docker image!
1) (optional) Test the newly created docker image
    1) `sudo docker run --rm -p 5896:5896 5ghackathon/laser_scan_unique:1.0.0`
    1) `python3 client.py`
1) Upload the newly created docker image to the dockerhub
    1) `sudo docker login`
        1) The credentials will be shared during the hackathon event
    1) `sudo docker push 5ghackathon/laser_scan_unique:1.0.0`


### Bonus task

If you are done with all tasks above, you may try the following bonus tasks.   


#### Process the LaserData in a separate Thread, instead of in the endpoint.  

This is the recommended approach because in this case, the data processing does not block the main thread, which is responsible for communication with the clients. You can use the available worker.py script to implement the "worker" and pass it data using the DataQueue in the TaskHolder. You can see the inspiration here: https://github.com/5G-ERA/Reference-NetApp/blob/06b97586615541a9e6d73e49f72975949e813f5d/src/python/era_5g_object_detection_standalone/era_5g_object_detection_standalone/interface.py#L109

1) Move the "core algorithm" to the "process_data" method in worker.py
2) Pass the scan_width parameter to the worker using metadata (you will need to modify the definition of the process_data method). 
   1) Remember! In this implementation, the worker process data from every connected robot. Therefore, you cannot save the parameter to the instance of the worker, because it could be different for each of the connected robots. 
3) Import the worker in the interface method, make an instance of it and run it.
4) Pass the data to the worker using the data_queue


## Consuming the Newly created Network Application by robots 

In this session, you should edit the original obstacle_avoidance ROS package to offload the computation to the network application.

1) Edit the detector_node.py from the obstacle_avoidance package.
    1) Remove the “core algorithm”
    1) In the __init__ method of NearestObject class, create an instance of the Python client, using the examples in client.py (without middleware, i.e., the network application must be started manually) and client_middleware.py (using the middleware, i.e., the network application must be onboarded).
    1) In the listener_callback, send the laser data to the network application using the send_json_http method of the Python client class. If you look at the data, you will notice that some ranges are "inf", which is not a json serializable value. Therefore, you need to go through the ranges list a replace any "inf" value with some large number (you can use the "range_max" property of the laser data, which specify the longest distance the laser can measure). You can see the example in client.py file.
    1) Create a callback method for results, in which you will publish the result using the distance_pub object. 
1) Run the nodes
    1) `source /opt/ros/foxy/setup.bash`
    1) `python3 detector_node `


### Bonus tasks

#### Control the robot from the network application

Modify the network application and the ROS nodes so the network application sends directly the speed information for the robot instead of sending the information of the nearest object. The ROS node will then publish the Twist message 