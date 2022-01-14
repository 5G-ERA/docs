# Performance test image

## Microk8s configuration
To functions correctly the Microk8s needs to have an additional modules installed: Multus and DNS. To install them use the command below.

```shell
microk8s enable dns multus
```

The example file to run the publisher and subscriber demo is placed in this directory, in [talker_listener_demo.yaml](talker_listener_demo.yaml) file. 

## Demo explanation 

The [talker_listener_demo.yaml](talker_listener_demo.yaml) file consists of 3 parts. 

### Network Attachment Definition

In this part we configure the additional Network Interface Card (NIC) for the pods to enable the pods to send and receive multicast requests. This is crucial as it enables the applications to connect using the DDS protocol. 

The configuration in the **NetworkAttachmentDefinition** has to be adjusted to the NIC details on the machine that Microk8s instance operates. 

In the config section the `"master"` property has to be configured with the name of the NIC to be used for example:  `"master": "enp0s3"`

Additionally the subnet has to be configured in the `"ranges"` node. The correct nodes configuration has to be set for the `subnet`, `rangeStart`, `rangeEnd` and `gateway` the set properties have to be compatible with the network settings your machine is running on.

The example configuration is shown below:
```json   
    "ranges": [
        [ {
            "subnet": "10.0.2.0/24",
            "rangeStart": "10.0.2.1",
            "rangeEnd": "10.0.2.254",
            "gateway": "10.0.2.255"
        } ]
    ]
```    

### Talker and Listener deployments

The talker and listener deployments specifies the configuration that will be used in the demo application. 

There are 2 parts of the configuration for each of the deployments correctly work in the demo. 

#### Annotations

The annotations are the way to configure the additional NIC for the pods and the deployments. We have to place the annotation info in the Deployment metadata node as well as in the Template metadata. It will result in the following configuration for the listener deployment:
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
name: ros-listener-deployment
labels:
    app: ros-listener
annotations:
    k8s.v1.cni.cncf.io/networks: dds-network
spec:
replicas: 1
selector:
    matchLabels:
    app: ros-listener
template:
    metadata:
    labels:
        app: ros-listener
    annotations:
        k8s.v1.cni.cncf.io/networks: dds-network
---
```

#### Container command

To run the container correctly the command has to be configured. The command is that keeps the container alive. After the command finishes, the container ends its lifespan. For both the talker and a listener the separate commands have to be specified. 

In the `command` node of the yaml file we specify that the command will use the shell to be run. The configuration is shown in the below example as `command: ["/bin/bash", "-c"]`. It tells the container to execute `bash` command and give it parameters. The parameters are what actually will be executed by the container. 

The parameters are defined in the `args` node. 
To enable the easy access to the ROS commands, the correct `setup.bash` file has to be sourced first. Next, we run a command to start the demo program of the talker and the listener. The example below showcases the command or the listener. For the talker, the `listener` word has to be changed for `talker`. 
```yaml
    spec:
      containers:
      - name: listener
        image: public.ecr.aws/c8q3f0b5/ros-performance-test:latest 
        command: ["/bin/bash", "-c"]
        args: ["source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp listener"]
        imagePullPolicy: IfNotPresent

```

The whole configuration is presented in the [talker_listener_demo.yaml](talker_listener_demo.yaml) file.

## Running demo with the performance tests
