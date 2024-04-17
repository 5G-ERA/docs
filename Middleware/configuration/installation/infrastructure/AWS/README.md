# AWS Infrastructure and k8s cluster configuration



This directory contains configuration files required to provision EKS Cluster with the required services.
This configuration is based on [Provision an EKS Cluster tutorial by Terraform](https://developer.hashicorp.com/terraform/tutorials/kubernetes/eks).


The Module configures the following aspects of the Middleware infrastructure:
* 3 public subnets
* 3 private subnets
* NAT Gateway for the private subnets
* EKS Cluster in the private subnets
* Auto-scaling node group
* Enables IRSA in EKS
* Configures IAM Role for Middleware to access k8s API
* Network Load Balancer
* Configures the created EKS Cluster for the Middleware and CentralApi deployment

## Prerequisites

To provide the necessary infrastructure the AWS Access Keys are needed. The machine needs to have the following tools installed:

* Terraform ([install guide](https://developer.hashicorp.com/terraform/tutorials/aws-get-started/install-cli))
* AWS CLI v2.7.0/v1.24.0 or newer ([install guide](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html), [configuration guide](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-quickstart.html))
* AWS IAM Authenticator ([install guide](https://docs.aws.amazon.com/eks/latest/userguide/install-aws-iam-authenticator.html))
* kubectl  v1.27.0 or newer ([install guide](https://kubernetes.io/docs/tasks/tools/))

## Provision

To provide the needed infrastructure, first, the terraform directory has to be initialized to download the necessary providers and modules. 

```shell
$ terraform init
```

Next, after everything is installed, we can start the planning process. 

```shell
$ terraform plan
```

Terraform will produce the plan that will be provisioned to reflect the desired infrastructure. At this point, nothing is created and the Plan proposed by Terraform will be 

To create the infrastructure, execute apply function:

```shell
$ terraform apply
```

Here, terraform will once again create the plan with the changes to the existing infrastructure. Before provision, you will be asked to confirm the creation by typing `yes` into the command line.

Afterward, the provisioning process will begin. It can take around 15-20 minutes, during which Terraform will constantly output the progress in the command line. 

## Accessing Kubernetes

When the provisioning process has been conducted and finished, we can configure access to the cluster with `kubectl`.

To configure the `kubectl` the information about the created cluster will be necessary. To access them we will use the `terraform output` command.

```shell
$ aws eks --region $(terraform output -raw region) update-kubeconfig \
    --name $(terraform output -raw cluster_name)
```

Afterwards, we can check if the cluster has been configured and if we have access to it.

```shell
$ kubectl cluster-info
```

```shell
$ kubectl get nodes
```

## Allowing access for other user groups
To enable the use of the Kubernetes cluster by other IAM users, the AWS auth `ConfigMap` needs to be updated. We will add permission to the already existing group with the following command:
```shell
$ eksctl create iamidentitymapping \
    --cluster 5G-ERA-AWS \
    --region eu-west-2 \
    --arn arn:aws:iam::132465798:role/KubernetesAdmin \
    --group system:masters \
    --no-duplicate-arns \
    --username admin-k8sAdminsGroup
```



## Destroy the infrastructure

To destroy the deployed infrastructure use `destroy` command.

```shell
$ terraform destroy
```

Once again, Terraform will plan each step and what changes will be made to the infrastructure. To confirm the destruction, type `yes` into the terminal.

The following process will look similar to the provisioning process, but instead, it will delete the created infrastructure. 
