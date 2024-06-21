# Infrastructure deployment with Terraform and CRoP Middleware installation for local Edge

The CRoP Middleware system relies on numerous backing services that have to be deployed before its own installation process, such as: `redis`, `influxdb`, `rabbitmq`, `ingress-nginx`, `loki/grafana`, and a customized `central-api` which is responsible for the CRoP Middleware registration and scalability management. When finishing this tutorial you should have CRoP Middleware deployed and connected to all the baking infrustructure. It is recomended to use the configurarion and deployment files that are linked to this tutorial for successful completion. Section ***6*** of the tutorial is optional, one should proceed with this section only for multiple Middleware deployments.

## Prerequisites

This installation tutorial assumes the CRoP Middleware is installed in the `Ubuntu` system with version `20.04` or newer.

The recommended hardware specifications for running the CRoP Middleware locally for testing purposes are:

* CPU - 4 cores
* RAM - 16 GB or above
* Memory - 100 GB or more

For the production environment, the hardware specifications will differ as the minimum requirements will be based on the number of Network Applications the Middleware will be running. The more Network Applications the more hardware resources will be needed.

## 1. Install Kubectl

The Kubectl is the command-line tool that allows communication and management of the Kubernetes cluster. To install it use the preferred way on the [official guide](https://kubernetes.io/docs/tasks/tools/install-kubectl-linux/), or run the following commands for installing kubectl binary with curl for Linux:\
#### 1.1 Download the latest release:
```shell
curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
```
#### 1.2 Validate the binary:
```shell
curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl.sha256"
```
#### 1.3 Validate against the checksum:
```shell
echo "$(cat kubectl.sha256)  kubectl" | sha256sum --check
```
#### 1.4 Install kubectl:
```shell
sudo install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl
```
#### 1.5 Verify installation:
```shell
kubectl version --client
```
#### 1.6 Afterward, if the `~/.kube` folder was not created:

```shell
mkdir -p ~/.kube
```

## 2. Install Microk8s

Microk8s is the minimal Kubernetes installation that can be used on the local computer. It will be used to run the Middleware. 

Although this guide provides installation instructions for the `microk8s`, any Kubernetes distribution like `minikube` or `kind` can be used. The installation instructions will differ and some configuration parts may differ in case of the deployment in a public cloud environment like AWS EKS or Azure AKS. We recomend using version 1.27 or 1.28 for microk8s.


#### 2.1 `microk8s` can be installed with the following command:

```shell
sudo snap install microk8s --classic --channel=1.27/stable
```

#### 2.2 After the installation is complete `microk8s` needs access to the `~/.kube` folder so we give it the permissions and we add the current user to the `microk8s` group, so we can use the commands without `sudo`.

```shell
sudo usermod -a -G microk8s $USER
sudo chown -f -R $USER ~/.kube
```

#### 2.3 After the installation is finished copy the configuration file of the `microk8s` to the `.kube/config` file for the `kubectl` command to be able to access the microk8s cluster. 

```shell
sudo microk8s config > ~/.kube/config
```

#### 2.4 Afterward, validate the connection to the cluster with the command

```shell
kubectl get all -n kube-system
```

With the `kubectl` access to the cluster, the additional modules have to be installed to ensure the correct work of the CRoP Middleware.

#### 2.5 We enable `metallb` addon to enable communication with the services inside the cluster with the dedicated IP address. During the execution of this command, **you will be asked to provide the ranges of the IP addresses you wish to use** for exposing services behind a Load Balancer, you can copy the suggested example excluding the quotes, like this one: `10.64.140.43-10.64.140.49,192.168.0.105-192.168.0.111`
```shell
 sudo microk8s enable metallb
```

#### 2.6 The DNS module is responsible for routing the DNS-based requests to be routed to the correct pods and services

```shell
 sudo microk8s enable dns
```

#### 2.7 We enable ingress as a backup for exposing the services from the cluster.
 **If you want to use the TLS encryption for your Middleware, you can skip enabling the ingress here and activate the already preconfigured `ingress-nginx` which includes the routes for the Middleware in the terraform `main.tf` by commenting out the resource. However, this will not automatically enable the TLS, its configuration will have to be addressed with your own domain and certificates.**.

```shell
 sudo microk8s enable ingress
```
#### 2.8 And lastly for the applications that require data persistance we enable the hostpath-storage addon.

```shell
sudo microk8s enable hostpath-storage
```

## 3. Install Terraform

Terraform is an Infrastructure as Code tool for provisioning and management of the infrastructure. To install Terraform use the following link: [official guide](https://developer.hashicorp.com/terraform/tutorials/aws-get-started/install-cli), or execute the following commands for Ubuntu/Debian:

#### 3.1 Ensure your system is up to date and the `gnupg`, `curl` and `software-properties-common` are installed:
```shell
sudo apt-get update && sudo apt-get install -y gnupg software-properties-common
```
#### 3.2 Install HashiCorp GPG key:
```shell
wget -O- https://apt.releases.hashicorp.com/gpg | \
gpg --dearmor | \
sudo tee /usr/share/keyrings/hashicorp-archive-keyring.gpg > /dev/null

```
#### 3.3 Verify key fingerprint:
```shell
gpg --no-default-keyring \
--keyring /usr/share/keyrings/hashicorp-archive-keyring.gpg \
--fingerprint

```
#### 3.4 Add the official HashiCorp repo to your system:
```shell
echo "deb [signed-by=/usr/share/keyrings/hashicorp-archive-keyring.gpg] \
https://apt.releases.hashicorp.com $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/hashicorp.list

```
#### 3.5 Download the package from HashiCorp:
```shell
sudo apt update
```
#### 3.6 Install Terraform:
```shell
sudo apt-get install terraform
```
#### 3.7 Verify installation:
```shell
terraform -help
```


### Initialize and deploy the CRoP infrastructure with Terraform

#### 3.8 Create a new directory in your system, this will be your terraform working directory:

```shell
mkdir terraform-crop
```

#### 3.9 Navigate to your newly created directory:
```shell
cd terraform-crop
```

#### 3.10 Copy the content from the [services](../services) directory in your newly created `terraform-crop` directory, make sure your directory contains the following files(`main.tf, influxdb-values.yaml, redis-values.yaml, loki-values.yaml, nginx-values.yaml, rabbitmq-values.yaml`) ***Note*** ***(Credentiasl for each service are in the `values.yaml` files, feel free to leave them as they are or modify them with your own prefference)*** then run the init command:
```shell
terraform init
```
#### 3.11 Make sure your configuration is valid and as expected with the plan command:

```shell
terraform plan
```
#### 3.12 Once the workspace has been initialize you can run the apply command `(the installation process will create namespaces for each baking service, to avoid installation failure make sure you don't have duplicate namespaces, check *main.tf* file for namespaces to be created)`, when you will be asked for confirmation type `yes` and press `ENTER`:
```shell
terraform apply
```
The picture below depicts all the backing service deployed with Terraform:

![Terraform deployment](/docs/Middleware/img/terraform-backing-services.png)

#### 3.13 Generate InfluxDB API token key
In your influxdb service, locate the NodePort that forwards to port `8086` and open a browser using these details, ex: `localhost:31996`, in first window click the GetStarted button and fill in your credentials as below:
> **Note:** If you left username and password unchanged in step 3.9 use the default `influxuser/influxpassword` otherwise use your own, however, for the organizartion use `5G-ERA` and for bucket name use `middleware` .

![Influx Authentication](/docs/Middleware/img/influx-aut.png)

#### 3.14 Copy the token key, you will need this in step 5.5 Middleware configuration
![Influx Authentication](/docs/Middleware/img/influx-auth.png)

## 4. Install Central-API
#### 4.1 Create namespace:
```shell
kubectl create namespace middleware-central
```
#### 4.2 The `central-api-edge.yaml` file is locate in the [Edge-middleware](Edge-middleware) directory, copy the file to your device and run the command:
```shell
kubectl apply -n middleware-central -f central-api-edge.yaml
```
The picture below depicts the Central-API:
![Central-API](/docs/Middleware/img/central-api.png)
## 5. Install CRoP Middleware
### Cluster configuration

After the `microk8s` is installed and the `kubectl` command has access to the cluster, it is time to configure the cluster so the middleware can be deployed and function correctly inside of it.

The files required for the execution of the cluster configuration are provided in the folder [crop-middleware](../../crop-middleware/)


**Note: To execute the provided commands, make sure you are in the same directory as the downloaded files.**

#### 5.1 For organization and segregation purposes Middleware works in a separate Kubernetes namespace. We create a new *namespace* with the following command: 

```shell
kubectl create namespace middleware
```

#### 5.2 For this purpose, the Service Account with the correct permissions is needed. The Service Account will give the necessary permissions to the Middleware for accessing the Kubernetes API and managing the resources as a part of its functionality.


To do so, use the command:
```shell
kubectl apply -f orchestrator_service_account.yaml 
```

#### 5.3 The next step is to create the Role, which specifies the permissions needed for the proper functioning of the Middleware. The `role` specifies the permissions to get, watch, list create and delete resources in the Middleware namespace. It affects the pods, services, deployments, namespaces, and replica sets in the cluster. To create the `Role`, use the following command:

```shell
kubectl apply -f orchestrator_role.yaml 
```

#### 5.4 The last step to configuring the Kubernetes cluster is to bind the Cluster Role to the Service Account.

For this `Role Binding` is necessary. To create it, use the following command:

```shell
kubectl apply -f orchestrator_role_binding.yaml
```

---

### 5.5 Middleware Configuration

The last step is to prepare the deployment script for the middleware. It can be found in the [Edge-middleware](Edge-middleware). In the `orchestrator-edge.yaml` file there are environment variables that must be set to ensure the correct work of the Orchestrator. 


The required variables are:

1. REDIS_INTERFACE_API_SERVICE_HOST - the address of the RedisInterface.API service, should be set to `redis-interface-api`.
2. REDIS_INTERFACE_API_SERVICE_PORT - the port of the RedisInterface.API service, should be set to `80`.
3. Middleware__Organization – the organization to which this middleware belongs. The organization is an artificial group of Middlewares that can cooperate.
4. Middleware__InstanceName – a **unique** name of the Middleware.
5. Middleware__InstanceType – Should be set to `Edge`.
6. Middleware__Address - The entry point for the middleware is usually the gateway address, however, for local deployment use the `http://` suffix, the ip of the machine, and the NodePort of the gateway service, ex: `http://192.168.50.23:31010`. 
7. CENTRAL_API_HOSTNAME - Address of the CentralAPI that is responsible for authenticating the Middleware instances during the startup. For more information refer to the [CentralAPI documentation](../../central-api)
8. CustomLogger__LoggerName - Either Loki/Elasticsearch.
9. CustomLogger__Url - The url of the logger.
10. CustomLogger__User - The username for the logger.
11. CustomLogger__Password - The password for the logger.
12. RabbitMQ__Address - The address of RabbitMQ. 
13. RabbitMQ__User - The user for RabbitMQ. 
14. RabbitMQ__Pass - The password for RabbitMQ.
15. Redis__ClusterHostname - The address of the redis backend.
16. Redis__Password - The password for the redis backend.
17. InfluxDB__Address - Address to which connect to InfluxDB, includes protocol, address, and port
18. InfluxDB__ApiKey - Api key to access InfluxDB

## Middleware version
The most up-to-date Middleware version is `v1.0.3`. Remember to set this tag in the `orchestrator-edge.yaml` file in the `spec -> template -> spec -> containers -> image`. 


## 5.5 Middleware deployment 

#### After all the values are set, the Middleware can be deployed. Start with the deployment of the Orchestrator:

```shell
kubectl apply -f orchestrator-edge.yaml -n middleware
```

The containers will be downloaded, and the Orchestrator will deploy the rest of the Middleware deployments and services required. The picture below depicts the CRoP Middleware:

![CRoP-Middleware](/docs/Middleware/img/crop-middleware.png)



## 6 Multiple Middleware deployment 
At this time you should have deployed the backing infrastructure and the CRoP Middleware system on your first Edge machine. If you want to deploy a second Middleware instance on a second Edge machine and synchronise it with the first instance of Middleware, here is what you should do in the second machine:

#### 6.1 Start by redoing the stept in sections 1, 2, 5, 5.1, 5,2, 5.3, and 5.4 in the exact same way, for preparing the environment.

#### 6.2 Following is step 5.5, however we need to modify the adresses of the services in the configuration file before we deploy the second Middleware.
Based on the type of the service, the address can contain the `http` suffix, the local name of the service `loki.loki.svc.cluster.local` and the port of the service `3100` which results in `http://loki.loki.svc.cluster.local:3100`, however, this will work if the loki service is deployed in the same machine where you deploy the first Middleware. For the second Middleware deployment the address will look a bit different, we need to use the IP of the first machine instead of the service name, and the NodePort instead of the port which will result in something similar to `http://192.168.50.23:31019`. ***NOTE (The structure af the addresses may differ, some may not need `http` suffix, some may not need `port` and so on, however the file that is linked below for the second Middleware deployment contains the exact addresses structure for each service)***
 Before even configuring the second Middleware deployment make sure that from the second machine you can actually access the services that were deployed in the first machine, by accessing the services as exaplined using the IP of the first machine and NodePort. The services you need to make sure can be accessed from the second machine are: `CentralAPI`, `Loki`, `RabbitMQ`, `Redis-cluster`, and `InfluxDB`. Here is an example configuration file for the second Middleware delpyment which is locate in [Edge-middleware](Edge-middleware), namely the `orchestrator-edge-two.yaml`.

 #### 6.2.1 After all the values are set, the second Middleware can be deployed:

```shell
kubectl apply -f orchestrator-edge-two.yaml -n middleware
```