## ROS & CROP

All hand-on experiences assumes you have Ubuntu 22.04 and ROS 2 Humble installed. Moreover, you need a functional docker instalation.

### 1) Demonstration: limitation of ROS2 DDS (BUT)
### 2) Hand on experience of remote object detection in unstructured networks (BUT)

In this section, you will try to run our object detector based on YOLO. You will learn the limitations of running such an application on CPU-only HW and compare the performance of different neural network models regarding consumed resources and acquired results. 

You will try to run the object detector on your computer and see the detection results on images from your web camera or arbitrary video file. Afterward, you will configure the Relay client to deploy an object detector network application on our GPU edge server or in the cloud and see the difference in the amount and quality of results.

First, you need to have a source of a video stream. The first option is to use your laptop's web camera: 

```bash 
ros2 run usb_cam usb_cam_node_exe
```

If the usb_cam package is not installed, you can install it like this:

```bash 
sudo apt install ros-humble-usb-cam
```

The command above creates a sensor_msgs/msg/Image topic named /usb_cam/image_raw. You can visualize it with the following command:

```bash 
ros2 run image_tools show_image --ros-args --remap /image:=/usb_cam/image_raw
```

If you don't want or cannot use the web camera, you can use an arbitrary video file (ideally with people, cars, or umbrellas) and make a topic from it with the following command (it will be on topic /image_raw):

```bash 
ros2 run image_publisher image_publisher_node video_file.mp4
```

The rest of the tutorial assumes you are using the web camera node, so if you are using the video file, you need to change the topic name to /image_raw

Now, when you have a working image topic with a video stream, you can run the object detector node:

```bash 
sudo docker run --rm -e INPUT_TOPIC=/usb_cam/image_raw -e OUTPUT_TOPIC=/results -e NETAPP_TORCH_DEVICE=cpu  but5gera/ros_object_detection:0.3.0
```

The INPUT_TOPIC variable sets the image topic the detector should use as an input. Similarly, the OUTPUT_TOPIC variable sets the topic's name with the results. The results topic is of type std_msgs/msg/String for simplicity. The NETAPP_TORCH_DEVICE=cpu tells the detector that CPU should be used for the detection (to use GPU, you need to have cuda and nvidia-docker installed and set the variable to cuda:0). By default, the detector will use the mobilenet model, which is suitable for CPUs and low-end GPU accelerators (such as Nvidia Jetson). To change the model to some more powerful variant, you can also set a NETAPP_MODEL_VARIANT=yolo_dark_net (that will really stress your CPU).

You can see the results with the ros2 topic command:

```bash 
ros2 topic echo /results
```

To see the detected bounding boxes, you can use the following docker image: 

```bash 
docker run --rm INPUT_TOPIC=/usb_cam/image_raw OUTPUT_TOPIC=/bounding_boxes but5gera/detection_publisher:1.0.0
```

It will create a /bounding_boxes topic that contains a video stream with superimposed bounding boxes that you can visualize with the show_image node:

```bash 
ros2 run image_tools show_image --ros-args --remap /image:=/bounding_boxes
```

To relieve stress from your computer, you can offload the object detector to some remote computer. In the simplest form, you will manually run your instance of an object detector in the remote node and configure your Relay client and server, which will forward the image topic to the server and the results back to your computer. 

To run the object detector on the shared server, you will utilize the following command (run it on the shared server through ssh):

```bash 
sudo docker run --rm -e INPUT_TOPIC=/usb_cam/image_raw -e OUTPUT_TOPIC=/results -e ROS_DOMAIN_ID=XX but5gera/ros_object_detection:0.3.0
```

You need to update the value of the ROS_DOMAIN_ID to be a number between 1-99, different from the other participants (you can use a random generator: https://xkcd.com/221/). You will use the same number in the configuration of the Relay server:

```bash 
docker run --rm --network host -e ROS_DOMAIN_ID=XX -e NETAPP_PORT=YYYYY --rm --network host -e TOPICS_FROM_CLIENT='[{"name": "/usb_cam/image_raw", "type": "sensor_msgs/msg/Image"}]' -e TOPICS_TO_CLIENT='[{"name": "/results", "type": "std_msgs/msg/String"}]' but5gera/ros2_relay_server:1.3.2
```

You need to adjust two variables: ROS_DOMAIN_ID to be the same as with the object detector and NETAPP_PORT to be a random number in the range 10000-20000 (and you will use this number when configuring the Relay client). The variables TOPICS_FROM_CLIENT and TOPICS_TO_CLIENT sets the topics to be mirrored from client to server and back.

On your laptop, you need to set up the Relay client:

```bash 
sudo docker run --rm --network host -e NETAPP_ADDRESS=http://address-of-server:YYYYY -e TOPICS_TO_SERVER='[{"name": "/usb_cam/image_raw", "type": "sensor_msgs/msg/Image"}]' -e TOPICS_FROM_SERVER='[{"name": "/results", "type": "std_msgs/msg/String"}]'  but5gera/ros2_relay_client:1.3.0
```

You must adjust the NETAPP_ADDRESS variable to contain the server's IP address and use the port number set in the previous step.

You can visualize results with the detection_publisher and show_image, just like with one of the previous steps.

Once you are satisfied with the results, you can utilize Middleware to deploy the application to the remote server:

```bash 
sudo docker run --network host --rm -e TOPICS_TO_SERVER='[{"name": "/usb_cam/image_raw", "type": "sensor_msgs/msg/Image"}]' -e TOPICS_FROM_SERVER='[{"name": "/results", "type": "std_msgs/msg/String"}]' -e USE_MIDDLEWARE=true -e MIDDLEWARE_USER=GUID_USER -e MIDDLEWARE_PASSWORD=PASS -e MIDDLEWARE_TASK_ID=TBA -e MIDDLEWARE_ADDRESS=server-ip:server-port -e MIDDLEWARE_ROBOT_ID=ROBOT-ID but5gera/ros2_relay_client:1.3.0
```

To instruct the Middleware to deploy the network application for you, you need to set the USE_MIDDLEWARE variable to true and set user, password, middleware address, and robot-id variables. In this case, the Middleware deploys the application to the relevant computer and sets up the communication channel between the two instances of the Relay. 
 
Once again, you can visualize results with the detection_publisher and show_image, just like with one of the previous steps.

### 3) Hand on experience of obstacle avoidance in unstructured networks (BUT)

In this section, you will try to deploy collision avoidance, a.k.a Forward Collision Warning Service Network Application (FCW). Contrary to the previous example, ROS is not used in this case. The complete documentation for the FCW is located here: https://github.com/5G-ERA/CollisionWarningService

For the following tutorial, only the client part is needed since the Network Application itself will be deployed on the remote server. To try the FCW, you will need a video from a robot or an autonomous vehicle, a calibration file and a configuration script for the corresponding video. Here you can download both: TBA

To install the client package, follow these steps:

```bash
python3 -m venv myvenv
myvenv\Scripts\activate
pip install fcw-client
```
Now you can run the example client script:

```bash
export NETAPP_ADDRESS=TBA
fcw_client_python_simple -c config.yaml --camera video.yaml video.mp4
```
In the previous section, the address of the deployed network application needs to be added. Ask the organizer where the network application is deployed.

To deploy the network application using the middleware, you can use a second client script, which requires setting several middleware variables. The organizers will provide you with the details of the configuration:

```bash
export MIDDLEWARE_ADDRESS=TBA
export MIDDLEWARE_USER=TBA
export MIDDLEWARE_PASSWORD=TBA
export MIDDLEWARE_TASK_ID=TBA
export MIDDLEWARE_ROBOT_ID=TBA
fcw_client_python_middleware -c config.yaml --camera video3.yaml video3.mp4
```

#### Running remote visualization

The visualisation should be enabled with config arguments during initialization command (`CollisionWorker` created 
with `viz` parameter set to `True` (ZeroMQ port can be also configured)).

If the FCW service has been started, **run RTSP server first**, e.g. https://github.com/bluenviron/mediamtx
on address: rtsp://localhost:8554 (TCP port 8554) and then:

```bash
docker run --rm -it -p 8554:8554 -e MTX_PROTOCOLS=tcp bluenviron/mediamtx:latest-ffmpeg
```

In other terminal run

```bash
cd fcw-service/fcw_service
fcw_visualization
```

or executing 
```bash
cd fcw-service/fcw_service
python3 visualization.py
```

You can view video, e.g. (localhost can be replaced with server address):

```bash
ffplay rtsp://localhost:8554/video
```

### 4) Demonstration: Behind scene story, fully orchestrated infrastructure automation (BED)

Current enviroment in robotics:

Strucuted Lab/environment
-QoS is fixed
-Network topology is static
-Single domain
-Single administration

Real enviroment:

Unstructured real-world settings:
-QoS changes constantly
-network topology is dynamic
-Mutlti domain co-exists
-multi admin

CROP provides an ecosystem and development environment for robot developers to build large scale distributed robotics.
Enable 5G enchaned cloud robotics to adapt robots automatically over the difital world.

-Ensure intelligent robot deployment in dynamic iunstructured network enviroments. 
-Software controlled instantiation of virtual networks meeting the requirements of the vertical.
-Adaptive autonomy on delivering the frictionless integration between the robot deployment and digital environment.
