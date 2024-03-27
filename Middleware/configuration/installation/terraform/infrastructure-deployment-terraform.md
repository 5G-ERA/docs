# Infrastructure deployment with Terraform and CROP Middleware installation

The CROP Middleware system relies on numerous backing services that have to be deployed before its own installation process, such as: `redis`, `influxdb`, `rabbitmq`, `ingress-nginx`, `loki/grafana`, and a customized `central-api` which is responsible for the CROP registration and scalability management.

## Prerequisites

This installation tutorial assumes the CROP Middleware is installed in the `Ubuntu` system with version `20.04` or newer.

The recommended hardware specifications for running the CROP Middleware locally for testing purposes are:

* CPU - 4 cores
* RAM - 16 GB or above
* Memory - 100 GB or more

For the production environment, the hardware specifications will differ as the minimum requirements will be based on the number of Network Applications the Middleware will be running. The more Network Applications the more hardware resources will be needed.

## Install Kubectl

The Kubectl is the command-line tool that allows communication and management of the Kubernetes cluster. To install it use the preferred way on the [official guide](https://kubernetes.io/docs/tasks/tools/install-kubectl-linux/).

Afterward, if the `~/.kube` folder was not created:

```shell
mkdir -p ~/.kube
```

## Install Microk8s

Microk8s is the minimal Kubernetes installation that can be used on the local computer. It will be used to run the Middleware. 

Although this guide provides installation instructions for the `microk8s`, any Kubernetes distribution like `minikube` or `kind` can be used. The installation instructions will differ and some configuration parts may differ in case of the deployment in a public cloud environment like AWS EKS or Azure AKS.


`microk8s` can be installed with the following command:

```shell
sudo snap install microk8s --classic
```

After the installation is complete `microk8s` needs access to the `~/.kube` folder so we give it the permissions and we add the current user to the `microk8s` group, so we can use the commands without `sudo`.

```shell
sudo usermod -a -G microk8s $USER
sudo chown -f -R $USER ~/.kube
```

After the installation is finished copy the configuration file of the `microk8s` to the `.kube/config` file for the `kubectl` command to be able to access the microk8s cluster. 

```shell
sudo microk8s config > ~/.kube/config
```

Afterward, validate the connection to the cluster with the command

```shell
kubectl get all -n kube-system
```

With the `kubectl` access to the cluster, the additional modules have to be installed to ensure the correct work of the CROP Middleware.

We enable `metallb` addon to enable communication with the services inside the cluster with the dedicated IP address. During the execution of this command, **you will be asked to provide the ranges of the IP addresses you wish to use** for exposing services behind a Load Balancer.
```shell
 sudo microk8s enable metallb
```

The DNS module is responsible for routing the DNS-based requests to be routed to the correct pods and services

```shell
 sudo microk8s enable dns
```

In the end, we enable ingress as a backup for exposing the services from the cluster. 

```shell
 sudo microk8s enable ingress
```

## Install Terraform

Terraform is an Infrastructure as Code tool for provisioning and management of the infrastructure. To install Terraform use the following link: [official guide](https://developer.hashicorp.com/terraform/tutorials/aws-get-started/install-cli).

### Initialize and deploy the CROP infrastructure with Terraform

Create a new directory in your system, this will be your terraform working directory:

```shell
mkdir terraform-crop
```

Navigate to your newly created directory:
```shell
cd terraform-crop
```

Copy the content from the `terraform` directory in your newly created `terraform-crop` directory, make sure your directory contains the following files(`main.tf, influxdb-values.yaml, redis-values.yaml, loki-values.yaml, nginx-values.yaml, rabbitmq-values.yaml`) and run the init command:
```shell
terraform init
```

Once the workspace has been initialize you can run the apply command, when you will be asked for confirmation type `yes` and press `ENTER`:
```shell
terraform apply
```

## Install Central-api
Create namespace:
```shell
kubectl create namespace middleware-central
```
Deploy the central-api. The `central-api.yaml` file is locate in the central-api directory, copy the file to your device and run the command:
```shell
kubectl apply -n middleware-central -f central-api.yaml
```
## Install CROP Middleware
### Cluster configuration

After the `microk8s` is installed and the `kubectl` command has access to the cluster, it is time to configure the cluster so the middleware can be deployed and function correctly inside of it.

The files required for the execution of the cluster configuration are provided [here](../../k8s/cluster-config).


**Note: To execute the provided commands, make sure you are in the same directory as the downloaded files.**

For organization and segregation purposes Middleware works in a separate Kubernetes namespace. We create a new *namespace* with the following command: 

```shell
kubectl create namespace middleware
```

For this purpose, the Service Account with the correct permissions is needed. The Service Account will give the necessary permissions to the Middleware for accessing the Kubernetes API and managing the
resources as a part of its functionality.


To do so, use the command:
```shell
kubectl apply -f orchestrator_service_account.yaml 
```

The next step is to create the Role, which specifies the permissions needed for the proper functioning of the Middleware. The `role` specifies the permissions to get, watch, list create and delete resources in the Middleware namespace. It affects the pods, services, deployments, namespaces, and replica sets in the cluster. To create the `Role`, use the following command:

```shell
kubectl apply -f orchestrator_role.yaml 
```

The last step to configuring the Kubernetes cluster is to bind the Cluster Role to the Service Account.

For this `Role Binding` is necessary. To create it, use the following command:

```shell
kubectl apply -f orchestrator_role_binding.yaml
```

---

### Middleware Configuration

The last step is to prepare the deployment script for the middleware. It can be found in the crop-middleware folder. In the `orchestrator.yaml` file there are environment variables that must be set to ensure the correct work of the Orchestrator. 


The required variables are:

1. IMAGE_REGISTRY – contains the address of the registry in which the Middleware images are stored. By default this value is `ghcr.io/5g-era`
2. Middleware__Organization – the organization to which this middleware belongs. The organization is an artificial group of Middlewares that can cooperate.
3. Middleware__InstanceName – a **unique** name of the Middleware.
4. Middleware__InstanceType – Either Edge/Cloud.
5. Middleware__Address - The entry point for the middleware is usually the gateway address ex: 10.10.18.17:80, for local deployment use the ip address of the machine ex: 192.168.50.80 
6. CustomLogger__LoggerName - Either Loki/Elasticsearch.
7. CustomLogger__Url - The url of the logger.
8. CustomLogger__User - The username for the logger.
9. CustomLogger__Password - The password for the logger.
10. Slice__Hostname - The hostname of the SliceManager API that allows integration of the 5G slices into the planning process of the Middleware.
11. RabbitMQ__Address - The address of RabbitMQ. 
12. RabbitMQ__User - The user for RabbitMQ. 
13. RabbitMQ__Pass - The password for RabbitMQ.
14. CENTRAL_API_HOSTNAME - Address of the CentralAPI that is responsible for authenticating the Middleware instances during the startup. For more information refer to the [CentralAPI documentation](CentralApi).
15. AWS_ACCESS_KEY_ID - Aws access key ID used to access the services in AWS like Secret Manager.
16. AWS_SECRET_ACCESS_KEY - Aws secret used to authenticate the access key.
17. Redis__ClusterHostname - The address of the redis backend.
18. Redis__Password - The password for the redis backend.
19. InfluxDB__Address - Address to which connect to InfluxDB, includes protocol, address, and port
20. InfluxDB__ApiKey - Api key to access InfluxDB

## Middleware version
The most up-to-date Middleware version is `v0.10.0`. Remember to set this tag in the `orchestrator.yaml` file in the `spec -> template -> spec -> containers -> image`. 

Until the Middleware releases version `1.0`, we recommend using the `latest` tag, as it is not guaranteed to provide backward compatibility. From versions `1.0` and later, backward compatibility will be ensured.

## Middleware deployment 

After all the values are set, the Middleware can be deployed. Start with the deployment of the Orchestrator:

```shell
kubectl apply -f orchestrator.yaml -n middleware
```

Alternatively, you can use utility scripts located at [k8s/orchestrator](../../k8s/orchestrator/):

```shell
./deploy.sh
```

The containers will be downloaded, and the Orchestrator will deploy the rest of the Middleware deployments and services required. 