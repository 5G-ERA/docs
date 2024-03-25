Juju installation and configuration
==================================

Juju is the Charmed Operator framework that allows to manage applications and automate the platform tasks. It supports the whole range of _Day 2_ operations. Juju as an Operator Framework manages not only Kubernetes instances but is also capable of analogical management of the hybrid clouds and bare metal server instances.

# Installation 

Juju installation should be dependent on the use case. The installation will cover the configuration of the local development environment using Juju with Microk8s. It will also cover the process of connecting Juju to the remote Kubernetes cluster as well as the Kubernetes cluster on the AWS. 

## Local development environment
The local Development environment instructions are supposed to give the full guide to how to configure the installation not only to have the Juju installed but also to be able to develop Charmed Operators that are used by Juju.

If you wish to just install Juju and then proceed to the configuration with the remote, skip the prerequisites section.

### Prerequisites
To configure the local environment for the charmed operators' development the 2 prerequisites have to be met:

* Charmcraft
* LXD
* Microk8s (or other minimal Kubernetes installation)

Charmcraft can be installed using the following `snap` command:
```
$ sudo snap install charmcraft
```
LXD is the program that is used by Charmcraft to build the Charmed Operators in the containerised environment, specified by the charmed operator configuration.

To set up the LXD follow the commands

```
$ sudo snap install lxd
$ sudo adduser $USER lxd
$ newgrp lxd
$ lxd init --auto
```
Microk8s (optional) - you should be able to use different minimal installations of the Kubernetes or the full version installed on the remote machine.

To install Microk8s follow these commands:

```
$ sudo snap install --classic microk8s

$ sudo adduser $USER microk8s

$ sudo microk8s status --wait-ready

# Enable the 'storage' and 'dns' addons
# (required for the Juju controller)
$ sudo microk8s enable storage dns ingress

# Alias kubectl so it interacts with MicroK8s by default
$ sudo snap alias microk8s.kubectl kubectl

$ newgrp microk8s
```

---
### Juju

Juju can be installed via the `snap` package using the following command:
```
$ sudo snap install juju --classic
```
After this step, we have a fully working instance of Juju. Now let's move to how to connect it to the Kubernetes instance.

---

## Connecting Juju 

### Local Microk8s instance
If you have decided to use Juju with Microk8s then the last step configuration has to be made. 

This configuration allows Juju to connect to the Microk8s instance and create the new namespace inside of which it will work.
```
# 'micro' is the name of the controller, you can change it for whatever suits you
$ juju bootstrap microk8s micro

# 'development' is also user-specified.
$ juju add-model dev
```
---

### Kubernetes/Microk8s instance in the local network

To connect Juju to the Kubernetes cluster located in the local network, the `kubeconfig` file has to be provided. 

You can copy the `kubeconfig` file from the remote server using the following command:

```
scp username@host:~/.kube/config .kube/config.remote
```
It will copy the `kubeconfig` file from the remote server, located in the `~/.kube/config` to the new file in your machine and name it `config.remote`.

Next, we can add use Juju to register remote k8s cloud by specifying the path to the `kubeconfig` file and bootstrapping it.

```
juju add-k8s myKubeCloudName < ~/.kube/config.remote

juju bootstrap myKubeCloudName
```

After this, we should be able to create a new model and deploy our application to the remote Kubernetes cluster.

---

### AWS cluster

To connect Juju to the AWS cluster make sure you have installed `AWS CLI` and `eksctl` on the machine you have Juju installed.

For the latest instructions on how to install `AWS CLI` and `eksctl` use the following instructions in the [AWS documentation](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-getting-started.html) and the following guide for the [eksctl utility](https://docs.aws.amazon.com/eks/latest/userguide/eksctl.html).

After the configuration of the `AWS CLI` and `eksctl`, we can proceed to the installation of the `kubectl` the [Kubernetes client](https://kubernetes.io/docs/tasks/tools/).
If you have followed the instructions with the installation of Microk8s in the local development environment, the **alias will conflict with the kubectl client**. A small adjustment will have to be made.

To keep using the local environment with the Microk8s and `kubectl` to access AWS, you can configure different aliases in your `.bashrc` or `.bash_aliases` file. You can add an alias to use Microk8s Kubernetes client using this command:

```
alias microctl='microk8s.kubectl'
```
After adding the entry to one of the files restart your terminal window or use the `bash` command.

Now you can use commands `kubectl` and `microctl` to access the AWS Kubernetes and 

As the next step, configure `AWS CLI`. You will have to provide the **AWS Access Key ID** and **AWS Secret Access Key**. Additionally, you will be asked to provide the default region that `AWS CLI` will operate on. 
```
$ aws configure
```

We have configured `AWS CLI` we can use it to get the `kubeconfig` file for the existing cluster in the cloud
```
$ aws eks --region <your-region-code> update-kubeconfig --name <your-cluster-name>
``` 

It will update an existing file or create the new one if it does not exist.

Then use the following command to get all the pods in the cluster:
```
$ kubectl get pods
```

After this command has run successfully you should see the contents of your cluster.

With all the dependencies prepared, we can connect Juju to the AWS cloud.

First, add a new cloud:

```
$ juju add-k8s <name-for-your-AWS-cluster-connection>
$ juju bootstrap <name-for-your-AWS-cluster-connection>
```
After this step, the new namespace in the AWS Cluster should be created. Now we can add a new model to juju and start working after this.

To add new Juju model use 
```
$ juju add-model dev
```

---

## AWS Cloud
This part of the document focuses on connecting to the whole AWS Cloud instead of a single AWS EKS Cluster. 

To connect to the AWS cloud some additional prerequisites have to be met.

### EC2 Full Access Account

To correctly provision required EC2 instances Juju uses an account with full EC2 access. Start by creating such a user in the AWS IAM panel. 

When creating the group for such an account remember to add it to the group with the at least `AmazonEC2FullAccess` policy.

After the user is created, download the credentials and store them in the secured location. They also have to be provided for Juju, so it can connect to the AWS cloud. 

To set up the credentials for the Juju use the following command:
```
$ juju add-credential aws
```
This command sets up the credentials to the AWS cloud. After entering it we will be asked do we wish to add them only to the client, to the currently active controller, or both. As we are setting up the new cloud, we are interested in adding this to the client only. 

In the next step, we will be asked to select the region that the credentials are to be used for. After selecting the desired region, we have to provide an access key and access secret that have been downloaded in the `.csv` file from the AWS Management Console, after the user have been created.

For additional information about supplying credentials to Juju or alternative ways of conducting this task, refer to the [official documentation](https://juju.is/docs/olm/credentials#heading--use-a-yaml-file).

---
### Bootstrap the cloud

After the user has been created we can move to bootstrap the cloud. To do this run the following command:

```
$ juju bootstrap <public-cloud-name>/<cloud-region> <controller-name>

# the example command will look like this

$ juju bootstrap aws/eu-west-1 aws-full-5G-ERA 
```
This command configures the following aspects of the cloud:
* The cloud to bootstrap - in our case this is AWS
* The region that we want to work on - by default it is `us-east-1` with the `/eu-west-1` we specify that we want to use a different region
* The name of the controller we want to provision

After entering this command, Juju will provision a new EC2 instance. By default this instance has `t3a.large` type. It means that it utilizes 2 cores and 8GB of RAM.

To specify the desired size of a deployed instance the following parameter has to be used: `--constraints="instance-type=t3.small"`. This command will result in deploying the desired service with the `t3.small` type unit.

---