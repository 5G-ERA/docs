# 1) Consuming Objected detection - Reference NetApp in machine with Middleware and ROS Melodic installed using USB CAM

In this turotial, we will launch an object recognition net application using 5G-ERA Middleware and consume it by using as input a ROS topic with a USB Camera.
In this way, by using RVIZ or image_view package, the bounding boxes of predictions can be depicted.

## 1. Install Middleware in Ubuntu 18.04.6 LTS

The following [guide](../Administrator/Middleware_Installation.md) show case the procedure.

## 2. Run python script to launch reference netApp.

Navigate to the script location under middleware-main/src/

```console
python3 MiddlewareInterfacePythonSnippet.py 
```

After this, a new pod will be created with the netApp. Use this command to check the state of the middleware namespace.

```console
watch -c kubectl get all -n middleware
```

## 3. Launch USB CAM ROS package

Plug your usb camera or usb inbuilt camera of your pc. If the ROS USB_CAM is not installed in your system, use this command:

```console
sudo apt-get install ros-melodic-usb-cam
```
**NOTE:** Change the ROS distro if you are using another ROS.

Launch the ROS package and verify it is working.

```console
rosrun usb_cam usb_cam_node 
```

You can verify it is working by running this command with the USB camera topic.
```console
rosrun image_view image_view image:=/usb_cam/image_raw
```

## 4. Launch NetApp ROS Client

You may docker pull, the ROS net application client:

```console
docker pull but5gera/noetic_client:1.2.0
```

Check the external IP address of the pod with the netApp. This will be the input parameter to "NETAPP_ADDRESS". The port will be done one of the network application.

Finally, include the camera topic in "INPUT_TOPIC". The command should look like:

```console
sudo docker run --rm --net host --env NETAPP_ADDRESS=10.64.140.44 --env NETAPP_PORT=5896 --env INPUT_TOPIC=/usb_cam/image_raw --env OUTPUT_TOPIC=/results but5gera/noetic_client:1.0.0
```

## 5. Retreive the preditions: 
Lets run image_view package with the output topic of the ROS netApp Client.

```console
rosrun image_view image_view image:=/results
```

<p align="left">
    <img src="imgs/pred.png" alt="- ">
</p>


# 2) Consuming Object detection - Reference NetApp in machine with ROS connected to a real Robot - Aisoy KiK

In this turotial, we will launch an object recognition net application using 5G-ERA Middleware and let a robot (Aisoy) consume the netApp. To visualice what the prediction images that the robot has, we will connect another PC with ROS Indigo to the robot and visualice the results with image_view package.

*This tutorial assumes you have completed section 1: Consuming Objected detection - Reference NetApp in machine with Middleware and ROS Melodic installed using USB CAM*

## 1. Assuming the middleware is deployed, we will launch a python script to deploy the object detection netApp.

```console
python3 MiddlewareInterfacePythonSnippet.py 
```

We may visualice the status of the deployed netApp with:

```console
watch -c kubectl get all -n middleware
```

## 2. Expose gateway and netApp:

Now for other elements in the network to access the middleware and the netApp, both services need to be exposed.

For the gateway, we will do so by launching the command:

```console
kubectl expose deployment gateway --type=NodePort --port=80 --name=gateway-nodeport -n middleware
```

For the netApp:


```console
kubectl expose deployment netapp-object-detection --type=NodePort --port=31565 --name=netapp-object-detection-nodeport -n middleware
```

The ports used are the ones from the ports section when running the command: *watch -c kubectl get all -n middleware*

## 3. Connect other PC ROS to the Robot ROS:

For doing so, we will follow the [official](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) web tutorial of ROS.

In this case, for Aisoy in this tutorial:

```console
echo ROS_IP
```

ROS_IP=192.168.1.61

```console
echo ROS_MASTER_URI
```

ROS_MASTER_URI=http://192.168.1.61:11311

In etc/hosts you mabe need to add the route to the other PC with its IP and hostname:

192.168.1.36    adrian-AO756

**All these values are configured before launching ROSCORE in the robot.**

In Aisoy, launch the robot core system:

```console
roslaunch airos_launch airos.launch
```

On the PC side:
```console
export ROS_IP=192.168.1.36 
```

```console
export ROS_MASTER_URI=http://192.168.1.61:11311
```

In etc/hosts you mabe need to add the route to the robot with its IP and hostname:

192.168.1.61    aisoy1

Now if we do, rostopic list we should see the topics of the robot including the camera, which for Aisoy is: /airos/camera/image

## 4. Launch netApp client:

In this case, the command NETAPP_ADDRESS is the IP address of the PC in which middleware is running. NETAPP_PORT is the port in NodePort of the deployed netApp. INPUT_TOPIC is the camera topic of the robot. OUTPUT_TOPIC it can be whatever name we want. We will also set ROS_MASTER_URI so the container can access the ROS topics of the robot. The version of netApp client is 1.2.0 

```console
sudo docker run --rm --net host --env NETAPP_ADDRESS=192.168.1.5 --env NETAPP_PORT=32349 --env INPUT_TOPIC=/airos/camera/image --env OUTPUT_TOPIC=/results --env ROS_MASTER_URI=http://192.168.1.61:11311 but5gera/noetic_client:1.2.0
```

## 5. Visualice the result predictions:

```console
rosrun image_view image_view image:=/results
```

<p align="left">
    <img src="imgs/bottle.png" alt="- ">
</p>


<p align="left">
    <img src="imgs/suitcase.png" alt="- ">
</p>
