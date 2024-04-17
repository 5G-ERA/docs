terraform {

  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = ">= 4.48.0"
    }
    tls = {
      source  = "hashicorp/tls"
      version = "~> 4.0.4"
    }

    cloudinit = {
      source  = "hashicorp/cloudinit"
      version = "~> 2.2.0"
    }

    kubernetes = {
      source  = "hashicorp/kubernetes"
      version = ">= 2.16.1"
    }
  }

  required_version = "~> 1.3"
}
provider "aws" {
  region = var.region
}
provider "kubernetes" {
  host                   = module.infrastructure.cluster_endpoint
  cluster_ca_certificate = base64decode(module.infrastructure.cluster_ca_certificate)
  exec {
    api_version = "client.authentication.k8s.io/v1beta1"
    command     = "aws"
    args = [
      "eks",
      "get-token",
      "--cluster-name",
      var.cluster_name
    ]
  }
}
locals {
  kubernetes_namespace = "middleware"
}

module "infrastructure" {
  source                          = "./modules/Infrastructure"
  region                          = var.region
  vpc_name                        = var.vpc_name
  cluster_name                    = var.cluster_name
  iam_policy_name                 = var.iam_policy_name
  iam_role_name                   = var.iam_role_name
  kubernetes_service_account_name = var.kubernetes_service_account_name
  aws_iam_role_arn = var.aws_iam_role_arn
}

module "cluster_config" {
  source = "./modules/k8s-config"

  region = var.region

  cluster_name           = module.infrastructure.cluster_name
  cluster_endpoint       = module.infrastructure.cluster_endpoint
  cluster_ca_certificate = module.infrastructure.cluster_ca_certificate

  service_account_name = var.kubernetes_service_account_name
  middleware_role_arn  = module.infrastructure.middleware_role_arn
  aws_iam_role_arn = var.aws_iam_role_arn
  depends_on           = [module.infrastructure]
}