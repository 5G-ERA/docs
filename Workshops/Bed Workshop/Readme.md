# BED-Workshop
Contents for the BED Workshop.

## Repository structure

The Workshop is divided into the 5 stages. Each stage has a following branch that contains the contents of of all the workshop until the currently selected stage. In example: stage 3 consists of information from stages 1 and 2, and the information of stage 3. 

This allows to easily transition from first step to another, with the main branch having the end look of the repository. 

The workshop consists of following stages:

1. Performance test Docker image preparation
2. ROS2 Deployment with Kubernetes
3. Performance tests using the built image
4. OSM deployment
5. Middleware introduction
## Backstory 

The scenario behind the workshop supports the idea behind the middleware. It allows having a clear idea of what process is needed to effectively communicate between the middleware and a robot. 

The middleware is the system that handles the communication between the robots and the 5G-ERA system. It is responsible for receiving the information from the robots and translating requests from robots. The communication between the robot and the system allows for coordinating the resources needed for a robot to execute the desired tasks. 

The middleware starts with the signal from the robot to execute a task. When the robot notifies the middleware about the task it wants to execute, middleware translates it into the action plan and deploys the necessary services to be used by the robot during the execution of the steps that the robot has to make. 
The robot is notified to execute specified steps with the support of the deployed services. It executes them in the sequential order defined in the Semantic DB knowledge base for the specified task. The order of the tasks guides it to the end goal which is the execution of the desired task.

The story presented in the workshop focuses on the deployment of the required services in the middleware. It will guide through the reasoning process behind a middleware and how the steps in the workshop correlate to the work that has to be done by the middleware.

## Stage 1 - Docker image preparation
This stage describes the process of building the performance_test image for 5G-ERA that is originally created by ApexAI. 

There are 2 ways of building the image. The first one is to execute the command that will automate the steps required.

1) Clone repository
```shell
git clone https://github.com/5G-ERA/BED-Workshop.git
```

2) Give permission to the /performance_image/build.sh

```shell
chmod 777 /performance_image/build.sh
```
3) Run the bash script

```shell
$ ./performance_image/build.sh
```
The script in the `performance_image` folder clones the ApexAI performance test repository, checks out to the correct branch and builds the specified `dockerfile` with the image.

After executing the `build.sh` command see the available images in the docker.

```shell
$ docker image ls
```

## Stage 2 
This stage will guide through the deployment of the ROS2 talker and a listener demo. 

The scenario assumes that part of the work being done by the middleware will be a deployment of the services to the kubernetes cluster. 

As on of the perquisites for ROS2 applications to be able to communicate with each other is to enable the multicast support. The multicast is not enabled by default in Kubernetes and the additional network card is needed. For this task, the local instance of Microk8s, a minimal K8s client, has `multus` addon installed.

Multus, when configured, enables the additional network card inside of the specified pods in the Kubernetes cluster.

To configure the additional network interface card in the pods use the `multus_config.yaml` file located in the `ros2_deployment` folder. Enter this folder.

```shell
$ cd ros2_deployment
```

In the `multus_config.yaml` for the correct work, the name of the `master` node has to be set to name of the network card that has an access to the internet on your machine. 

Use `ifconfig` command to retrieve all the names of the NICs in your system.

```shell
$ ifconfig
```
After the `master` property is set, update the IP address ranges to reflect the configuration in your NIC.

When done, apply the changes to the kubernetes cluster.

```shell
$ kubectl apply -f multus_config.yaml
```
### Listener deployment

With the config installed, create the listener deployment as in the `listener.yaml` file. Note the following part of the file:

```yaml
kind: Deployment
metadata:
  name: ros-listener-deployment
  labels:
    app: ros-listener
  annotations:
    k8s.v1.cni.cncf.io/networks: ros-network
```
The `annotations` part defines that the deployment needs to specify the name of the network created in the previous step. 

```shell
$ kubectl apply -f listener.yaml
```

With the listener deployed the logs can be followed by the using the following command: (L in lower case used here)

```shell
$ kubectl logs --follow -l app=ros-listener
```

### Talker deployment

The process of deployment of the talker image is almost the same as for the listener. The file differs in name, labels used to locate the service and in the command used to start the container. 

Also note that the configuration of the additional network interface is still necessary. Remember to check if the additional NIC is assigned to the deployment.


Deploy the talker deployment with the following command.

```shell
$ kubectl apply -f talker.yaml
```



## Stage 3 - Performance image functionality

### Digital Twin with k8, Webots & ROS2 Web simulation:
In this experiment, it will be demostrated how to create a k8 pod with an integrated simulation streaming at port 1234 and visualice the world and robot using web browser. This approach will enable scalable and containered simulations integrated with ROS2 systems and with online web visualization.

1) Setup – Installation of MicroK8s with configured Multus.

```
-sudo snap install microk8s --classic
-sudo usermod -a -G microk8s $USER
-sudo chown -f -R $USER ~/.kube
-sudo microk8s enable dns multus
```
2) Download yaml k8 recipy for digital twin.
```
git clone https://github.com/5G-ERA/BED-Workshop.git
```
The file for the digital twin yaml is named: talker_listener_bk_running.yaml

3) Launch the experiment:

```
sudo microk8s.kubectl apply -f talker_listener_bk_running.yaml
```

4) Review the status of the deployment: 

```
sudo microk8s.kubectl get all
```

5) Exec into the pod and launch the turtlebot 3 experiment. The containerID can be either ros-listener-deplyment-id/ ros-talker-deployment-id

```
sudo microk8s.kubectl exec --stdin --tty containerID -- /bin/bash
```

Inside the container, source ROS2 opt default installation and source the ROS2 workspace created via the yaml file.

ROS2 foxy installation
```
source /opt/ros/foxy/setup.bash
```
Workspace
```
source ~/dev_ws/install/setup.bash
```
Launch the up with virtual screen.
download xvfb first by this command: 
instructions can be found here: https://linuxhint.com/install-xvfb-ubuntu/
Some more commands can be needed for proper installation 
apt-get update --fix-missing (to fix any issues missing or broken)
sudo apt-get install firefox 
```
sudo apt install xvfb
sudo apt-get install ros-foxy-webots-ros2
ros2 launch webots_ros2_universal_robot multirobot_launch.py 
xvfb-run ros2 launch webots_ros2_turtlebot robot_launch.py
```

6) Visualice the simulation in the web browser. Webots will use port 1234 for the streaming. It has already been mapped in the deployed yaml file with a k8 service.

```
http://containerIP:1234/index.html
```
7) Control the robot with teleop ROS2 package.

Inside the k8's pod, launch this command to install the teleop package.

```
sudo apt-get install ros-foxy-teleop-twist-keyboard
```

8) Launch the teleop node:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Visualice the robot in the web simulation moving according to the cmd_vel commands published by the teleop node.

![imagen](https://user-images.githubusercontent.com/26432703/154964739-07195e9e-5042-482c-aa4c-44f3a7a406d1.png)


You can clean your enviroment after finishing the experiment by removing the k8 pods:
```
sudo microk8s.kubectl delete deployment weboot2
```
```
sudo microk8s.kubectl delete service ros-listener
```

Double check by listing all pods running:

```
sudo microk8s.kubectl get all
```

This experiment uses Webots ROS2 interface developed by: https://github.com/cyberbotics/webots_ros2
For more information about how to run the same example in docker, visit: https://cyberbotics.com/doc/guide/using-ros

### Measure performance of your ROS2 systems and topologies:

For this purpose, two tools will be used. ApexAI performance is the most commmon package to use for testing your system and evaluate a variety of different metrics. ApexAI allows to configure different DDS technologies, QoS, and datatypes. Some exaple of datatypes allows: Array1k, PointCloud512k, RadarTrack...

Additionally, the ROS2_np_latency will be used to test various ROS2 topologies and the latency.

#### ApexAI performance test: 
For this experiment, the Performance_Test.yaml will be used. 

1) Launch the yaml recipy. This will create two pods connected with a ROS2 network of type Vlan.

```
sudo microk8s.kubectl apply -f Performance_Test.yaml
```

2) Exec into either one of the pods.

```
sudo microk8s.kubectl exec --stdin --tty containerID --/bin/bash
```

Both containers have the ApexAI tools and the ROS2_np_latency. Let's run a ApexAI test. The ApexAI executable is in the path: 

```
/opt/performance_test/lib/performance_test/perf_test 
```
The test to run will use DDS fastRTPS, output the results to a csv file and send a message of type Array1k.

```
/opt/performance_test/lib/performance_test/perf_test -c  FastRTPS --msg Array1k --output csv
```

Stop with control+c when you want to stop the test, there is no limit time. An example of the output is available in this repo with the name: Performance_APEXIA_EXAMPLE_OUTPUT.csv

For an extended documentation of ApexAI tools:
https://drive.google.com/file/d/15nX80RK6aS8abZvQAOnMNUEgh7px9V5S/view
ApexAI is available for a variety of ROS2 distributions.
https://gitlab.com/ApexAI/performance_test

To get the list of all available datatypes to run performance test, use:
```
/opt/performance_test/lib/performance_test/perf_test -c  FastRTPS –msg-list
```
#### ROS2_mp_Latency:

The interesting idea behing ros2_mp_lantecy is that it allows to test many topologies of your ROS2 system by creating and deploying fully c++ written ROS2 nodes. In this way, the simmilarity to the real scenario is closer. This package uses 3 types of nodes: sources, work and sink. 1) Sources will publish the data at a given rate and with specific datatype. Work node's represent any ros2 node that works with the data (object recognition for example) and sinks are the nodes that study the latency of the data within all the jumps. You can have as many jumps and topologies as desired. For more information, visit offical tool git: https://github.com/neil-rti/ros2_mp_latency 

![image](https://user-images.githubusercontent.com/26432703/154928105-33b5bfc4-dc55-4070-b3fe-c928318222df.png)

For this experiment, we will launch in one pod a source node and in another pod a sink node. We will exec into the container and run the source and sink nodes.
In the talker pod, run:
```
ros2 run mp_latency ipcsource_1kb &
```
In the listener pod, run:
```
ros2 run mp_latency ipcsink_1kb &
```

The output will provide a histogram, a log and a stadistics file. For a detailed explanation of the results, visit: 
https://www.rti.com/blog/latest-connext-dds-ros-2-performance-benchmarks

## Stage 4 - OSM deployment


To clone the example packages to be deployed in the OSM use the following commands.

```shell
$ git clone https://osm.etsi.org/gitlab/vnf-onboarding/osm-packages.git

# unzip the package 

$ tar xvvf osm-packages-master.tar.gz

```
The example packages will be used to deploy the KNF.

To deploy the KNF first some preparations will have to be made.

### Change the Openstack network name

The essential part to making work the deployment is the adjustment of the default network created by the Openstack. 

Retrieve the Openstack password using the command

```shell
$ sudo snap get microstack config.credentials.keystone-password


```

Copy the password and login to the OpenStack panel available at `192.168.56.4`. The username is admin.

After the successful login, navigate to the `Network > Network Topology > Topology` tab and edit the virtual network located on the right-hand side.

Click the network on the topology diagram and then click the `Edit Network` button. Change the name to `mgmt-net`.

### Add K8s cluster

Next switch to the Edge machine and export the Kubernetes configuration.

```shell
$ microk8s.config > kubeconfig.yaml
```

Copy it to the OSM machine and add the new cluster using configuration.

```shell
$ osm k8scluster-add --creds kubeconfig.yaml \
                     --version "1.23" --vim openstack-site\
                     --k8s-nets '{"k8s_net1": "mgmt-net"}' \
                     --description "K8s cluster" my-k8s-cluster
```
When the Kubernetes cluster is added, proceed to add example Virtual network Functions and Network services. OpenLDAP will be used as an example. 
Navigate to the downloaded git repository. Locate the `openldap_knf` and `openldap_ns` folders.

Starting with the first one, edit the `openldap_vnfd.yaml` file to the following contents:

```yaml
vnfd:
  description: KNF with single KDU using a helm-chart for openldap version 1.2.3
  df:
  - id: default-df
  ext-cpd:
  - id: mgmt-ext
    k8s-cluster-net: mgmt-net
  id: openldap_knf
  k8s-cluster:
    nets:
    - id: mgmt-net
  kdu:
  - name: ldap
    helm-chart: stable/openldap
  mgmt-cp: mgmt-ext
  product-name: openldap_knf
  provider: Telefonica
  version: '1.0'
```
Keep in mind to maintain the `k8s-cluster-net` property in pair with the name of the network specified while creating the cluster.

Exit the folder and enter the `openldap_ns` folder.

Edit the `openldap_nsd.yaml` file to the following contents.

```yaml
nsd:
  nsd:
  - description: NS consisting of a single KNF openldap_knf connected to mgmt network
    designer: OSM
    df:
    - id: default-df
      vnf-profile:
      - id: openldap
        virtual-link-connectivity:
        - constituent-cpd-id:
          - constituent-base-element-id: openldap
            constituent-cpd-id: mgmt-ext
          virtual-link-profile-id: mgmt-net
        vnfd-id: openldap_knf
    id: openldap_ns
    name: openldap_ns
    version: '1.0'
    virtual-link-desc:
    - id: mgmt-net
      mgmt-network: 'true'
    vnfd-id:
    - openldap_knf
```
When specifying the `vnfd-id` set is the same as the id of KNF.

Exit the folder. To instantiate the KNF and NS use the following commands

```shell
$ osm nfpkg-create openldap_knf

$ osm nspkg-create openldap_ns
```
Create a new Network Service instance with the following command 

```shell
$ osm ns-create --ns_name ldap --nsd_name openldap_ns \
                --vim_account openstack-site 
```
After logging into the OSM account, and navigating to the `Instances > NS Instances` the new Network Service should be visible during the deployment process or already deployed.


## Stage 5 - Middleware considerations

The middleware is the system placed in between the Robot and the services required by the robot.
### Middleware functionality

The middleware as software is responsible for communication with robots to allow them to have all the resources needed to execute the necessary tasks. It handles the process of receiving the request from the robot, translating the predefined task definition into the action plan. The middleware plans the necessary steps to be executed and the services needed. With the ready plan, it deploys the necessary services. 

When the services are deployed the Robot is informed about the first step that has to be executed and receives the address for the services to be utilized. 

When the robot finishes the current step, is informed about the next one. 

### Middleware assumptions

To make the functionality reliable some assumptions have to be met.

Before the system can handle the communication with the Robot it has to be deployed in the environment that is supposed to work in. While the OSM as the deployment manager is assumed to be deployed only once, the rest of the middleware will be deployed in each location. This means all the Edges and potentially in the cloud.

When the Robot is started it has to inform the system about the task it wants to execute. 
Such information reaches the Middleware and it prepares the Action Plan for the Robot and the Resource plan with the services needed for the robot to execute the desired steps. With the plan prepared and services deployed, the Robot receives the information with the next step to execute, with the services to be utilized, to complete the whole task. 
The robot is supposed to continuously inform the middleware about its status. To make the task easier, the robot will have a service installed on-board that will handle the communication with the system. The software of the robot will have to communicate with the program installed on the robot.
The Robot will also have to respond to the middleware requests to execute the steps given by the middleware. The whole paradigm is similar to the ROS2 Action Server.

The tasks will have to be defied in the Redis Knowledge Base with the necessary steps to execute the task. 

The Middleware will monitor the state of the services used and will inform the Robot if any of them has encountered the error or has been terminated. In the case of prolonged issues with the service, the Middleware should schedule the re-plan and notify the robot about the change of the plan or the availability of the resources. 

During the whole work of the Middleware, the system should keep the up to date services and Robots topology using represented in form of a graph.

After the service is no longer in use by any of the Robots, the Middleware will terminate it.

### Software reliability

Reliability is one of the key principles of good software. So should the 5G-ERA Middleware try to achieve.
But the reliability has to come from the whole system. Even if some components fail, the system should remain functional. 

To ensure reliability the tasks should have implemented the backup plans in case some services will fail. 
The backup plans will be executed when the middleware will fail in restoring the service. During the unavailability of the service, the robot will be notified about the change in the service status. If the service is not available and the re-plan has not solved the issue, the Robot can be informed that the system cannot provide the necessary resources and the calculations should temporarily be conducted by the Robot itself. 
The process of recovering from the service errors should be planned for each service, as different services can have other procedures for the recovery.
When the system recovers from the error caused by one of the systems it should inform the Robot about the availability of the recovered services. 
