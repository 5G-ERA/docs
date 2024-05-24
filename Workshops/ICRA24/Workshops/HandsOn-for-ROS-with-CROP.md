## ROS & CROP

All hand-on experiences assumes you have Ubuntu 22.04 and ROS 2 Humble installed. Moreover, you need a functional docker instalation.



### 1) Demonstration: limitation of ROS2 DDS (BUT)
### 2) Hand on experience of remote object detection in unstructured networks (BUT)

In this section, you will try to run our object detector based on YOLO. You will learn the limitations of running such an application on CPU-only HW and compare the performance of different neural network models regarding consumed resources and acquired results. 

You will try to run the object detector on your computer and see the detection results on images from your web camera or arbitrary video file. Afterward, you will configure the Relay client to deploy an object detector network application on our GPU edge server or in the cloud and see the difference in the amount and quality of results.

First, you need to have a source of a video stream. The first option is to use your laptop's web camera: 

```bash 
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="mjpeg2rgb"
```

If the usb_cam package is not installed, you can install it like this:

```bash 
sudo apt install ros-humble-usb-cam
```

The command above creates a sensor_msgs/msg/Image topic named /image_raw. You can visualize it with the following command:

```bash 
ros2 run image_tools showimage --ros-args --remap /image:=/image_raw
```

If you have a problem with your camera not working properly, check the official documentation of the [usb_cam](https://github.com/ros-drivers/usb_cam/tree/ros2) package.

If you don't want or cannot use the web camera, you can use an arbitrary video file (ideally with people, cars, or umbrellas) and make a topic from it with the following command (it will be on topic /image_raw):

```bash 
ros2 run image_publisher image_publisher_node video_file.mp4
```

Now, when you have a working image topic with a video stream, you can run the object detector node:

```bash 
sudo docker run --rm --network host -e INPUT_TOPIC=/image_raw -e OUTPUT_TOPIC=/res -e NETAPP_TORCH_DEVICE=cpu but5gera/ros2_object_detection:0.3.0
```

The INPUT_TOPIC variable sets the image topic the detector should use as an input. Similarly, the OUTPUT_TOPIC variable sets the topic's name with the results. The results topic is of type std_msgs/msg/String for simplicity. The NETAPP_TORCH_DEVICE=cpu tells the detector that CPU should be used for the detection (to use GPU, you need to have cuda and nvidia-docker installed and set the variable to cuda:0). By default, the detector will use the mobilenet model, which is suitable for CPUs and low-end GPU accelerators (such as Nvidia Jetson). 

To change the model to some more powerful variant, you can also set a NETAPP_MODEL_VARIANT=yolo_dark_net.

```bash 
docker run --rm -e INPUT_TOPIC=/image_raw -e OUTPUT_TOPIC=/res -e NETAPP_TORCH_DEVICE=cpu -e NETAPP_MODEL_VARIANT=yolo_dark_net but5gera/ros2_object_detection:0.3.0
```

You can try some more pre-prepared models with the NETAPP_MODEL_VARIANT variable:

```
NETAPP_MODEL_VARIANT=yolox_tiny
NETAPP_MODEL_VARIANT=yolo_dark_net
NETAPP_MODEL_VARIANT=yolo_dark_net2
NETAPP_MODEL_VARIANT=mask_rcnn_r50
```


You can see the results with the ros2 topic command:

```bash 
ros2 topic echo /res
```

To see the detected bounding boxes, you can use the following docker image: 

```bash 
docker run --rm --network host -e INPUT_IMAGES=/image_raw -e OUTPUT_IMAGES=/bounding_boxes -e RESULTS=/res but5gera/ros2_detection_publisher:1.0.0
```

It will create a /bounding_boxes topic that contains a video stream with superimposed bounding boxes that you can visualize with the show_image node:

```bash 
ros2 run image_tools showimage --ros-args --remap /image:=/bounding_boxes
```

To relieve stress from your computer, you can offload the object detector to some remote computer. In the simplest form, you will manually run your instance of an object detector in the remote node and configure your Relay client and server, which will forward the image topic to the server and the results back to your computer. 

To run the object detector on the shared server, you will utilize the following command (run it on the server through ssh). 

```bash 
docker run --rm -e INPUT_TOPIC=/image_raw -e OUTPUT_TOPIC=/res --gpus all but5gera/ros2_object_detection:0.3.0
```

Now you should run the relay server. Optionally, you can specify the port the server should run on:

```bash 
docker run --rm --network host -e NETAPP_PORT=5896 -e TOPICS_FROM_CLIENT='[{"name": "/image_raw", "type": "sensor_msgs/msg/Image"}]' -e TOPICS_TO_CLIENT='[{"name": "/res", "type": "std_msgs/msg/String"}]' but5gera/ros2_relay_server:1.5.0
```

The variables TOPICS_FROM_CLIENT and TOPICS_TO_CLIENT sets the topics to be mirrored from client to server and back.

On your laptop, you need to set up the Relay client:

```bash 
docker run --rm --network host -e NETAPP_ADDRESS=http://SERVER_ADDRESS:5896 -e TOPICS_TO_SERVER='[{"name": "/image_raw", "type": "sensor_msgs/msg/Image"}]' -e TOPICS_FROM_SERVER='[{"name": "/res", "type": "std_msgs/msg/String"}]'  but5gera/ros2_relay_client:1.5.0
```

You must adjust the NETAPP_ADDRESS variable to contain the server's IP address and use the port number set in the previous step.

You can visualize results with the detection_publisher and show_image, just like with one of the previous steps.

Once you are satisfied with the results, you can utilize Middleware to deploy the application to the remote server:

```bash 
sudo docker run --network host --rm -e TOPICS_TO_SERVER='[{"name": "/image_raw", "type": "sensor_msgs/msg/Image"}]' -e TOPICS_FROM_SERVER='[{"name": "/res", "type": "std_msgs/msg/String"}]' -e USE_MIDDLEWARE=true -e MIDDLEWARE_USER=GUID_USER -e MIDDLEWARE_PASSWORD=PASS -e MIDDLEWARE_TASK_ID=GUID_TASK -e MIDDLEWARE_ADDRESS=SERVER_ADDRESS:PORT -e MIDDLEWARE_ROBOT_ID=GUID_ROBOT but5gera/ros2_relay_client:1.5.0
```

To instruct the Middleware to deploy the network application for you, you need to set the USE_MIDDLEWARE variable to true and set user, password, middleware address, and robot-id variables. In this case, the Middleware deploys the application to the relevant computer and sets up the communication channel between the two instances of the Relay. 
 
Once again, you can visualize results with the detection_publisher and show_image, just like with one of the previous steps.

### 3) Hand on experience of obstacle avoidance in unstructured networks (BUT)

In this section, you will try to deploy collision avoidance, a.k.a Forward Collision Warning Service Network Application (FCW). Contrary to the previous example, ROS is not used in this case. The complete documentation for the FCW is located here: https://github.com/5G-ERA/CollisionWarningService

For the following tutorial, only the client part is needed since the Network Application itself will be deployed on the remote server. To try the FCW, you will need a video from a robot or an autonomous vehicle, a calibration file and a configuration script for the corresponding video.

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
export MIDDLEWARE_ADDRESS=SERVER_ADDRESS:PORT
export MIDDLEWARE_USER=GUID_OF_USER
export MIDDLEWARE_PASSWORD=PASSWORD
export MIDDLEWARE_TASK_ID=GUID_OF_TASK
export MIDDLEWARE_ROBOT_ID=GUID_OF_ROBOT
fcw_client_python_middleware -c config.yaml --camera video.yaml video.mp4
```

#### Running remote visualization

The visualisation should be enabled with config arguments during initialization command (`CollisionWorker` created 
with `viz` parameter set to `True` (ZeroMQ port can be also configured)).B y default, visualization is turned on 
using `viz` parameter in CollisionWarningClient.

If the FCW service has been started, **run RTSP server first**, e.g. https://github.com/bluenviron/mediamtx
on address: rtsp://localhost:8554 (TCP port 8554) and then:

```bash
docker run --rm -it -p 8554:8554 -e MTX_PROTOCOLS=tcp bluenviron/mediamtx:latest-ffmpeg
```

In other terminal...

If you don't have fcw-service installed, install it and run it
```bash
pip install fcw-service
fcw_visualization
```

or use git
```bash
git clone https://github.com/5G-ERA/CollisionWarningService.git
cd CollisionWarningService
cd fcw-service
pip install .
cd fcw_service
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
